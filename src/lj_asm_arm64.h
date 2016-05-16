/*
** ARM IR assembler (SSA IR -> machine code).
** Copyright (C) 2005-2016 Mike Pall. See Copyright Notice in luajit.h
*/

/* -- Register allocator extensions --------------------------------------- */

/* Allocate a register with a hint. */
static Reg ra_hintalloc(ASMState *as, IRRef ref, Reg hint, RegSet allow)
{
  Reg r = IR(ref)->r;
  if (ra_noreg(r)) {
    if (!ra_hashint(r) && !iscrossref(as, ref))
      ra_sethint(IR(ref)->r, hint);  /* Propagate register hint. */
    r = ra_allocref(as, ref, allow);
  }
  ra_noweak(as, r);
  return r;
}

/* Allocate a scratch register pair. */
static Reg ra_scratchpair(ASMState *as, RegSet allow)
{
    lua_unimpl();
   return 0;
}

#if !LJ_SOFTFP
/* Allocate two source registers for three-operand instructions. */
static Reg ra_alloc2(ASMState *as, IRIns *ir, RegSet allow)
{
    lua_unimpl();
   return 0;
}
#endif

/* -- Guard handling ------------------------------------------------------ */

/* Generate an exit stub group at the bottom of the reserved MCode memory. */
static MCode *asm_exitstub_gen(ASMState *as, ExitNo group)
{
  MCode *mxp = as->mcbot;
  int i;
  if (mxp + 4*4+4*EXITSTUBS_PER_GROUP >= as->mctop)
    asm_mclimit(as);
  /* str lr, [sp]; bl ->vm_exit_handler; .long DISPATCH_address, group. */
  *mxp++ = A64I_STR|A64F_D(RID_LR)|A64F_N(RID_SP);
  *mxp = A64I_BL|(((MCode *)(void *)lj_vm_exit_handler-mxp)&0x03ffffffu);
  mxp++;
  *mxp++ = (MCode)i32ptr(J2GG(as->J)->dispatch);  /* DISPATCH address */
  *mxp++ = group*EXITSTUBS_PER_GROUP;
  for (i = 0; i < EXITSTUBS_PER_GROUP; i++)
    *mxp++ = A64I_B|((-6-i)&0x00ffffffu);
  lj_mcode_sync(as->mcbot, mxp);
  lj_mcode_commitbot(as->J, mxp);
  as->mcbot = mxp;
  as->mclim = as->mcbot + MCLIM_REDZONE;
  return mxp - EXITSTUBS_PER_GROUP;
}

/* Setup all needed exit stubs. */
static void asm_exitstub_setup(ASMState *as, ExitNo nexits)
{
  // !!!TODO shared with ARM
  ExitNo i;
  if (nexits >= EXITSTUBS_PER_GROUP*LJ_MAX_EXITSTUBGR)
    lj_trace_err(as->J, LJ_TRERR_SNAPOV);
  for (i = 0; i < (nexits+EXITSTUBS_PER_GROUP-1)/EXITSTUBS_PER_GROUP; i++)
    if (as->J->exitstubgroup[i] == NULL)
      as->J->exitstubgroup[i] = asm_exitstub_gen(as, i);
}

/* Emit conditional branch to exit for guard. */
static void asm_guardcc(ASMState *as, A64CC cc)
{
  MCode *target = exitstub_addr(as->J, as->snapno);
  MCode *p = as->mcp;
  if (LJ_UNLIKELY(p == as->invmcp)) {
    as->loopinv = 1;
    *p = A64I_BL | ((target-p) & 0x03ffffffu);
    emit_branch(as, A64F_B_CC(A64I_B, cc^1), p+1);
    return;
  }
  /* ARM64 doesn't have conditional BL, so we emit an unconditional BL
     and branch around it with the opposite condition */
  emit_branch(as, A64I_BL, target);
  emit_branch(as, A64F_B_CC(A64I_B, cc^1), p+1);
}

/* -- Operand fusion ------------------------------------------------------ */

/* Limit linear search to this distance. Avoids O(n^2) behavior. */
#define CONFLICT_SEARCH_LIM	31

/* Check if there's no conflicting instruction between curins and ref. */
static int noconflict(ASMState *as, IRRef ref, IROp conflict)
{
    lua_unimpl();
    return 0;
}

/* Fuse the array base of colocated arrays. */
static int32_t asm_fuseabase(ASMState *as, IRRef ref)
{
    lua_unimpl();
    return 0;
}

/* Fuse array/hash/upvalue reference into register+offset operand. */
static Reg asm_fuseahuref(ASMState *as, IRRef ref, int32_t *ofsp, RegSet allow,
			  int lim)
{
  /* !!!TODO NFI what this does, the comment about LDRD below looks dodgy */
  lua_todo();
  IRIns *ir = IR(ref);
  if (ra_noreg(ir->r)) {
    if (ir->o == IR_AREF) {
      if (mayfuse(as, ref)) {
        if (irref_isk(ir->op2)) {
          IRRef tab = IR(ir->op1)->op1;
          int32_t ofs = asm_fuseabase(as, tab);
          IRRef refa = ofs ? tab : ir->op1;
          ofs += 8*IR(ir->op2)->i;
          if (ofs > -lim && ofs < lim) {
            *ofsp = ofs;
            return ra_alloc1(as, refa, allow);
          }
        }
      }
    } else if (ir->o == IR_HREFK) {
      if (mayfuse(as, ref)) {
        int32_t ofs = (int32_t)(IR(ir->op2)->op2 * sizeof(Node));
        if (ofs < lim) {
          *ofsp = ofs;
          return ra_alloc1(as, ir->op1, allow);
        }
      }
    } else if (ir->o == IR_UREFC) {
      if (irref_isk(ir->op1)) {
        GCfunc *fn = ir_kfunc(IR(ir->op1));
        int32_t ofs = i32ptr(&gcref(fn->l.uvptr[(ir->op2 >> 8)])->uv.tv);
        *ofsp = (ofs & 255);  /* Mask out less bits to allow LDRD. */
        return ra_allock(as, (ofs & ~255), allow);
      }
    }
  }
  *ofsp = 0;
  return ra_alloc1(as, ref, allow);
}

/* Fuse m operand into arithmetic/logic instructions. */
static uint32_t asm_fuseopm(ASMState *as, A64Ins ai, IRRef ref, RegSet allow)
{
  IRIns *ir = IR(ref);
  if (ra_hasreg(ir->r)) {
    ra_noweak(as, ir->r);
    return A64F_M(ir->r);
  } else if (irref_isk(ref)) {
    uint32_t k = emit_isk12(ai, ir->i);
    if (k == -1)
      return k;
  } else if (mayfuse(as, ref)) {
#if 0
    /* !!!TODO fuse shifts into this instruction, as ARM does */
    if (ir->o >= IR_BSHL && ir->o <= IR_BROR) {
      Reg m = ra_alloc1(as, ir->op1, allow);
      ARMShift sh = ir->o == IR_BSHL ? ARMSH_LSL :
		    ir->o == IR_BSHR ? ARMSH_LSR :
		    ir->o == IR_BSAR ? ARMSH_ASR : ARMSH_ROR;
      if (irref_isk(ir->op2)) {
	return A64F_M(m) | ARMF_SH(sh, (IR(ir->op2)->i & 31));
      } else {
	Reg s = ra_alloc1(as, ir->op2, rset_exclude(allow, m));
	return A64F_M(m) | ARMF_RSH(sh, s);
      }
    } else if (ir->o == IR_ADD && ir->op1 == ir->op2) {
      Reg m = ra_alloc1(as, ir->op1, allow);
      return A64F_M(m) | ARMF_SH(ARMSH_LSL, 1);
    }
#endif
  }
  return A64F_M(ra_allocref(as, ref, allow));
}

/* Fuse shifts into loads/stores. Only bother with BSHL 2 => lsl #2. */
static IRRef asm_fuselsl2(ASMState *as, IRRef ref)
{
    lua_unimpl();
    return 0;
}

/* Fuse XLOAD/XSTORE reference into load/store operand. */
static void asm_fusexref(ASMState *as, A64Ins ai, Reg rd, IRRef ref,
			 RegSet allow, int32_t ofs)
{
    lua_unimpl();
}

/* Fuse to multiply-add/sub instruction. */
static int asm_fusemadd(ASMState *as, IRIns *ir, A64Ins ai, A64Ins air)
{
    lua_todo();
    return 0;
}



/* -- Calls --------------------------------------------------------------- */

/* Generate a call to a C function. */
static void asm_gencall(ASMState *as, const CCallInfo *ci, IRRef *args)
{
    lua_unimpl();
}

/* Setup result reg/sp for call. Evict scratch regs. */
static void asm_setupresult(ASMState *as, IRIns *ir, const CCallInfo *ci)
{
    lua_unimpl();
}

static void asm_callx(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Returns ------------------------------------------------------------- */

/* Return to lower frame. Guard that it goes to the right spot. */
static void asm_retf(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Type conversions ---------------------------------------------------- */

#if !LJ_SOFTFP
static void asm_tointg(ASMState *as, IRIns *ir, Reg left)
{
    lua_unimpl();
}

static void asm_tobit(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}
#endif

static void asm_conv(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_strto(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Memory references --------------------------------------------------- */

/* Get pointer to TValue. */
static void asm_tvptr(ASMState *as, Reg dest, IRRef ref)
{
    lua_unimpl();
}

static void asm_aref(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* Inlined hash lookup. Specialized for key type and for const keys.
** The equivalent C code is:
**   Node *n = hashkey(t, key);
**   do {
**     if (lj_obj_equal(&n->key, key)) return &n->val;
**   } while ((n = nextnode(n)));
**   return niltv(L);
*/
static void asm_href(ASMState *as, IRIns *ir, IROp merge)
{
    lua_unimpl();
}

static void asm_hrefk(ASMState *as, IRIns *ir)
{
  IRIns *kslot = IR(ir->op2);
  IRIns *irkey = IR(kslot->op1);
printf("%p\n",irkey->ptr.ptr64);
  int32_t ofs = (int32_t)(kslot->op2 * sizeof(Node));
  int32_t kofs = ofs + (int32_t)offsetof(Node, key);
  Reg dest = (ra_used(ir) || ofs > 4095) ? ra_dest(as, ir, RSET_GPR) : RID_NONE;
  Reg node = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg key = RID_NONE, type = RID_TMP, idx = node;
  RegSet allow = rset_exclude(RSET_GPR, node);
  lua_assert(ofs % sizeof(Node) == 0);
  /* !!!TODO check 4095 for AArch64 */
  if (ofs > 4095) {
    idx = dest;
    rset_clear(allow, dest);
    kofs = (int32_t)offsetof(Node, key);
  } else if (ra_hasreg(dest)) {
    emit_opk(as, A64I_ADDx, dest, node, ofs, allow); /*!!!TODO w or x */
  }
  asm_guardcc(as, CC_NE);
  if (!irt_ispri(irkey->t)) {
    key = ra_scratch(as, allow);
    rset_clear(allow, key);
  }
  rset_clear(allow, type);
  if (irt_isnum(irkey->t)) {
    emit_ccmpk(as, A64I_CCMPw, CC_EQ, 0, type,
             (int32_t)ir_knum(irkey)->u32.hi, allow);
    emit_opk(as, A64I_CMPw, 0, key,
             (int32_t)ir_knum(irkey)->u32.lo, allow);
  } else {
printf("%p\n",irkey->ptr.ptr64);
    emit_ccmpk(as, A64I_CCMPw, CC_EQ, 0, type,
             (int32_t)ir_knum(irkey)->u32.hi, allow);
    emit_opk(as, A64I_CMNx, 0, type, -irt_toitype(irkey->t), allow);
  }
  emit_lso(as, A64I_LDRw, type, idx, kofs+4); /* !!!TODO w or x */
  if (ra_hasreg(key)) emit_lso(as, A64I_LDRw, key, idx, kofs); /* !!!TODO w or x */
  if (ofs > 4095)
    emit_opk(as, A64I_ADDx, dest, node, ofs, RSET_GPR);
}

static void asm_uref(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_fref(ASMState *as, IRIns *ir)
{
  UNUSED(as); UNUSED(ir);
  lua_assert(!ra_used(ir));
}

static void asm_strref(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Loads and stores ---------------------------------------------------- */

static A64Ins asm_fxloadins(IRIns *ir)
{
  switch (irt_type(ir->t)) {
  case IRT_I8: return /*ARMI_LDRSB*/0;
  case IRT_U8: return /*ARMI_LDRB*/0;
  case IRT_I16: return /*ARMI_LDRSH*/0;
  case IRT_U16: return /*ARMI_LDRH*/0;
  case IRT_NUM: lua_assert(!LJ_SOFTFP); return /*ARMI_VLDR_D*/0;
  case IRT_FLOAT: if (!LJ_SOFTFP) return /*ARMI_VLDR_S*/0;
  default: return /*ARMI_LDR*/0;
  }
}

static A64Ins asm_fxstoreins(IRIns *ir)
{
  switch (irt_type(ir->t)) {
  case IRT_I8: case IRT_U8: return /*ARMI_STRB*/0;
  case IRT_I16: case IRT_U16: return /*ARMI_STRH*/0;
  case IRT_NUM: lua_assert(!LJ_SOFTFP); return /*ARMI_VSTR_D*/0;
  case IRT_FLOAT: if (!LJ_SOFTFP) return /*ARMI_VSTR_S*/0;
  default: return /*ARMI_STR*/0;
  }
}

static void asm_fload(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg idx = ra_alloc1(as, ir->op1, RSET_GPR);
  A64Ins ai = asm_fxloadins(ir);
  int32_t ofs;
  if (ir->op2 == IRFL_TAB_ARRAY) {
    ofs = asm_fuseabase(as, ir->op1);
    if (ofs) {  /* Turn the t->array load into an add for colocated arrays. */
      emit_dn(as, (A64I_ADDx^A64I_BINOPk)|ofs, dest, idx);
      return;
    }
  }
  ofs = field_ofs[ir->op2];
  emit_lso(as, ai, dest, idx, ofs);
}

static void asm_fstore(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_xload(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_xstore_(ASMState *as, IRIns *ir, int32_t ofs)
{
    lua_unimpl();
}

#define asm_xstore(as, ir)	asm_xstore_(as, ir, 0)

static void asm_ahuvload(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_ahustore(ASMState *as, IRIns *ir)
{
  if (ir->r != RID_SINK) {
    RegSet allow = RSET_GPR;
    Reg idx, src = RID_NONE, type = RID_NONE;
    int32_t ofs = 0;
    if (irt_isnum(ir->t)) {
      src = ra_alloc1(as, ir->op2, RSET_FPR);
      idx = asm_fuseahuref(as, ir->op1, &ofs, allow, 1024); /* !!!TODO what is 1024 */
      emit_lso(as, A64I_STRd, src, idx, ofs);
    } else
    {
      if (!irt_ispri(ir->t)) {
        src = ra_alloc1(as, ir->op2, allow);
        rset_clear(allow, src);
      }
      type = ra_allock(as, (int32_t)irt_toitype(ir->t), allow);
      idx = asm_fuseahuref(as, ir->op1, &ofs, rset_exclude(allow, type), 4096);
      if (ra_hasreg(src))
	emit_lso(as, A64I_STRw, src, idx, ofs); /* !!!TODO STRx? */
      emit_lso(as, A64I_STRw, type, idx, ofs+4); /* !!!TODO STRx? */
    }
  }
}

static void asm_sload(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Allocations --------------------------------------------------------- */

#if LJ_HASFFI
static void asm_cnew(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}
#else
#define asm_cnew(as, ir)	((void)0)
#endif

/* -- Write barriers ------------------------------------------------------ */

static void asm_tbar(ASMState *as, IRIns *ir)
{
    lua_todo();
}

static void asm_obar(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Arithmetic and logic operations ------------------------------------- */

#if !LJ_SOFTFP
static void asm_fparith(ASMState *as, IRIns *ir, A64Ins ai)
{
    lua_unimpl();
}

static void asm_fpunary(ASMState *as, IRIns *ir, A64Ins ai)
{
    lua_unimpl();
}

static void asm_callround(ASMState *as, IRIns *ir, int id)
{
    lua_unimpl();
}

static void asm_fpmath(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}
#endif

static int asm_swapops(ASMState *as, IRRef lref, IRRef rref)
{
  IRIns *ir;
  if (irref_isk(rref))
    return 0;  /* Don't swap constants to the left. */
  if (irref_isk(lref))
    return 1;  /* But swap constants to the right. */
  ir = IR(rref);
  /* !!!TODO check for AArch64 fuseable ops here instead */
#if 0
  if ((ir->o >= IR_BSHL && ir->o <= IR_BROR) ||
      (ir->o == IR_ADD && ir->op1 == ir->op2))
    return 0;  /* Don't swap fusable operands to the left. */
  ir = IR(lref);
  if ((ir->o >= IR_BSHL && ir->o <= IR_BROR) ||
      (ir->o == IR_ADD && ir->op1 == ir->op2))
    return 1;  /* But swap fusable operands to the right. */
#endif
  return 0;  /* Otherwise don't swap. */
}

static void asm_intop(ASMState *as, IRIns *ir, A64Ins ai)
{
  IRRef lref = ir->op1, rref = ir->op2;
  Reg left, dest = ra_dest(as, ir, RSET_GPR);
  uint32_t m;
  /* !!!TODO AArch64 doesn't have RSB, so swapping is harder than ARM */
#if 0
  if (asm_swapops(as, lref, rref)) {
    IRRef tmp = lref; lref = rref; rref = tmp;
    if ((ai & ~A64I_S) == A64I_SUB || (ai & ~A64I_S) == A64I_SBC)
      ai ^= (A64I_SUB^A64I_RSB);
  }
#endif
  left = ra_hintalloc(as, lref, dest, RSET_GPR);
  m = asm_fuseopm(as, ai, rref, rset_exclude(RSET_GPR, left));
  if (irt_isguard(ir->t)) {  /* For IR_ADDOV etc. */
    asm_guardcc(as, CC_VS);
    ai |= A64I_S;
  }
  emit_dn(as, ai^m, dest, left);
}

static void asm_intop_s(ASMState *as, IRIns *ir, A64Ins ai)
{
  if (as->flagmcp == as->mcp) {  /* Drop cmp r, #0. */
    as->flagmcp = NULL;
    as->mcp++;
    ai |= A64I_S;
  }
  asm_intop(as, ir, ai);
}

static void asm_intneg(ASMState *as, IRIns *ir, A64Ins ai)
{
    lua_unimpl();
}

/* NYI: use add/shift for MUL(OV) with constants. FOLD only does 2^k. */
static void asm_intmul(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_add(ASMState *as, IRIns *ir)
{
  if (irt_isnum(ir->t)) {
    if (!asm_fusemadd(as, ir, A64I_FMADDd, A64I_FMADDd))
      asm_fparith(as, ir, A64I_ADDd);
    return;
  }
  asm_intop_s(as, ir, A64I_ADDx);
}

static void asm_sub(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_mul(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

#define asm_addov(as, ir)	asm_add(as, ir)
#define asm_subov(as, ir)	asm_sub(as, ir)
#define asm_mulov(as, ir)	asm_mul(as, ir)

#if !LJ_SOFTFP
#define asm_div(as, ir)		asm_fparith(as, ir, /*ARMI_VDIV_D*/0)
#define asm_pow(as, ir)		asm_callid(as, ir, IRCALL_lj_vm_powi)
#define asm_abs(as, ir)		asm_fpunary(as, ir, /*ARMI_VABS_D*/0)
#define asm_atan2(as, ir)	asm_callid(as, ir, IRCALL_atan2)
#define asm_ldexp(as, ir)	asm_callid(as, ir, IRCALL_ldexp)
#endif

#define asm_mod(as, ir)		asm_callid(as, ir, IRCALL_lj_vm_modi)

static void asm_neg(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

static void asm_bitop(ASMState *as, IRIns *ir, A64Ins ai)
{
    lua_unimpl();
}

#define asm_bnot(as, ir)	asm_bitop(as, ir, /*ARMI_MVN*/0)

static void asm_bswap(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

#define asm_band(as, ir)	asm_bitop(as, ir, /*ARMI_AND*/0)
#define asm_bor(as, ir)		asm_bitop(as, ir, /*ARMI_ORR*/0)
#define asm_bxor(as, ir)	asm_bitop(as, ir, /*ARMI_EOR*/0)

static void asm_bitshift(ASMState *as, IRIns *ir, int sh)
{
    lua_unimpl();
}

#define asm_bshl(as, ir)	asm_bitshift(as, ir, /*ARMSH_LSL*/0)
#define asm_bshr(as, ir)	asm_bitshift(as, ir, /*ARMSH_LSR*/0)
#define asm_bsar(as, ir)	asm_bitshift(as, ir, /*ARMSH_ASR*/0)
#define asm_bror(as, ir)	asm_bitshift(as, ir, /*ARMSH_ROR*/0)
#define asm_brol(as, ir)	lua_assert(0)

static void asm_intmin_max(ASMState *as, IRIns *ir, int cc)
{
    lua_unimpl();
}

static void asm_fpmin_max(ASMState *as, IRIns *ir, int cc)
{
    lua_unimpl();
}

static void asm_min_max(ASMState *as, IRIns *ir, int cc, int fcc)
{
    lua_unimpl();
}

//#define asm_min(as, ir)		asm_min_max(as, ir, CC_GT, CC_HI)
//#define asm_max(as, ir)		asm_min_max(as, ir, CC_LT, CC_LO)
#define asm_min(as, ir)		lua_unimpl()
#define asm_max(as, ir)		lua_unimpl()

/* -- Comparisons --------------------------------------------------------- */

/* Map of comparisons to flags. ORDER IR. */
static const uint8_t asm_compmap[IR_ABC+1] = {
  /* op  FP swp  int cc   FP cc */
  /* LT       */ CC_GE + (CC_HS << 4),
  /* GE    x  */ CC_LT + (CC_HI << 4),
  /* LE       */ CC_GT + (CC_HI << 4),
  /* GT    x  */ CC_LE + (CC_HS << 4),
  /* ULT   x  */ CC_HS + (CC_LS << 4),
  /* UGE      */ CC_LO + (CC_LO << 4),
  /* ULE   x  */ CC_HI + (CC_LO << 4),
  /* UGT      */ CC_LS + (CC_LS << 4),
  /* EQ       */ CC_NE + (CC_NE << 4),
  /* NE       */ CC_EQ + (CC_EQ << 4),
  /* ABC      */ CC_LS + (CC_LS << 4)  /* Same as UGT. */
};

/* FP comparisons. */
static void asm_fpcomp(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* Integer comparisons. */
static void asm_intcomp(ASMState *as, IRIns *ir)
{
  A64CC cc = (asm_compmap[ir->o] & 15);
  IRRef lref = ir->op1, rref = ir->op2;
  Reg left;
  uint32_t m;
  int cmpprev0 = 0;

  /* !!!TODO what does this mean, isu32 looks suspicious for 64bit arch */
  lua_assert(irt_isint(ir->t) || irt_isu32(ir->t) || irt_isaddr(ir->t));

  /* !!!TODO ARM exploits TST here for comp(BAND(left, right) */

  left = ra_alloc1(as, lref, RSET_GPR);

  m = asm_fuseopm(as, A64I_CMPx, rref, rset_exclude(RSET_GPR, left));
  asm_guardcc(as, cc);
  emit_n(as, A64I_CMPx^m, left);

  /* !!!TODO we should do this so that the compare can be elided by
     making the instruction before flagsetting */
  /* Signed comparison with zero and referencing previous ins? */
  if (cmpprev0 && (cc <= CC_NE || cc >= CC_GE))
    as->flagmcp = as->mcp;  /* Allow elimination of the compare. */

}

static void asm_comp(ASMState *as, IRIns *ir)
{
  if (irt_isnum(ir->t))
    asm_fpcomp(as, ir);
  else
    asm_intcomp(as, ir);
}

#define asm_equal(as, ir)	asm_comp(as, ir)

#if LJ_HASFFI
/* 64 bit integer comparisons. */
static void asm_int64comp(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}
#endif

/* -- Support for 64 bit ops in 32 bit mode ------------------------------- */

/* Hiword op of a split 64 bit op. Previous op must be the loword op. */
static void asm_hiop(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Profiling ----------------------------------------------------------- */

static void asm_prof(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

/* -- Stack handling ------------------------------------------------------ */

/* Check Lua stack size for overflow. Use exit handler as fallback. */
static void asm_stack_check(ASMState *as, BCReg topslot,
			    IRIns *irp, RegSet allow, ExitNo exitno)
{
    lua_unimpl();
}

/* Restore Lua stack from on-trace state. */
static void asm_stack_restore(ASMState *as, SnapShot *snap)
{
    lua_unimpl();
}

/* -- GC handling --------------------------------------------------------- */

/* Check GC threshold and do one or more GC steps. */
static void asm_gc_check(ASMState *as)
{
    lua_unimpl();
}

/* -- Loop handling ------------------------------------------------------- */

/* Fixup the loop branch. */
static void asm_loop_fixup(ASMState *as)
{
  MCode *p = as->mctop;
  MCode *target = as->mcp;
  if (as->loopinv) {  /* Inverted loop branch? */
    /* asm_guardcc already inverted the bcc and patched the final bl. */
    p[-2] |= ((uint32_t)(target-p+2) & 0x00ffffffu);
  } else {
    p[-1] = A64I_B | ((uint32_t)((target-p)+1) & 0x03ffffffu);
  }
}

/* -- Head of trace ------------------------------------------------------- */

/* Reload L register from g->cur_L. */
static void asm_head_lreg(ASMState *as)
{
    lua_unimpl();
}

/* Coalesce BASE register for a root trace. */
static void asm_head_root_base(ASMState *as)
{
    lua_unimpl();
}

/* Coalesce BASE register for a side trace. */
static RegSet asm_head_side_base(ASMState *as, IRIns *irp, RegSet allow)
{
    lua_unimpl();
    return 0;
}

/* -- Tail of trace ------------------------------------------------------- */

/* Fixup the tail code. */
static void asm_tail_fixup(ASMState *as, TraceNo lnk)
{
    lua_unimpl();
}

/* Prepare tail of code. */
static void asm_tail_prep(ASMState *as)
{
  /* !!!TODO shared with ARM */
  MCode *p = as->mctop - 1;  /* Leave room for exit branch. */
  if (as->loopref) {
    as->invmcp = as->mcp = p;
  } else {
    as->mcp = p-1;  /* Leave room for stack pointer adjustment. */
    as->invmcp = NULL;
  }
  *p = 0;  /* Prevent load/store merging. */
}

/* -- Trace setup --------------------------------------------------------- */

/* Ensure there are enough stack slots for call arguments. */
static Reg asm_setup_call_slots(ASMState *as, IRIns *ir, const CCallInfo *ci)
{
    lua_unimpl();
    return 0;
}

static void asm_setup_target(ASMState *as)
{
  // !!!TODO shared with ARM.
  /* May need extra exit for asm_stack_check on side traces. */
  asm_exitstub_setup(as, as->T->nsnap + (as->parent ? 1 : 0));
}

/* -- Trace patching ------------------------------------------------------ */

/* Patch exit jumps of existing machine code to a new target. */
void lj_asm_patchexit(jit_State *J, GCtrace *T, ExitNo exitno, MCode *target)
{
    lua_unimpl();
}

