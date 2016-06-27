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

/* Allocate two source registers for three-operand instructions. */
static Reg ra_alloc2(ASMState *as, IRIns *ir, RegSet allow)
{
  IRIns *irl = IR(ir->op1), *irr = IR(ir->op2);
  Reg left = irl->r, right = irr->r;
  if (ra_hasreg(left)) {
    ra_noweak(as, left);
    if (ra_noreg(right))
      right = ra_allocref(as, ir->op2, rset_exclude(allow, left));
    else
      ra_noweak(as, right);
  } else if (ra_hasreg(right)) {
    ra_noweak(as, right);
    left = ra_allocref(as, ir->op1, rset_exclude(allow, right));
  } else if (ra_hashint(right)) {
    right = ra_allocref(as, ir->op2, allow);
    left = ra_alloc1(as, ir->op1, rset_exclude(allow, right));
  } else {
    left = ra_allocref(as, ir->op1, allow);
    right = ra_alloc1(as, ir->op2, rset_exclude(allow, left));
  }
  return left | (right << 8);
}

/* -- Guard handling ------------------------------------------------------ */

/* Generate an exit stub group at the bottom of the reserved MCode memory. */
static MCode *asm_exitstub_gen(ASMState *as, ExitNo group)
{
  MCode *mxp = as->mcbot;
  int i;
  intptr_t dispatch;
  if (mxp + 5*4+4*EXITSTUBS_PER_GROUP >= as->mctop)
    asm_mclimit(as);
  dispatch = i64ptr(J2GG(as->J)->dispatch);
  /* str lr, [sp, #TMPDofs];
     bl ->vm_exit_handler;
     .long DISPATCH_address (lo)
     .long DISPATCH_address (hi)
     .long group. */
  *mxp++ = A64I_STRx|A64F_D(RID_LR)|A64F_N(RID_SP)|A64F_A(CFRAME_OFS_TMPD>>3);
  *mxp = A64I_BL|(((MCode *)(void *)lj_vm_exit_handler-mxp)&0x03ffffffu);
  mxp++;
  *mxp++ = (MCode)(dispatch & 0xffffffff);          /* DISPATCH address (lo) */
  *mxp++ = (MCode)((dispatch >> 32) & 0xffffffff);  /* DISPATCH address (hi) */
  *mxp++ = group*EXITSTUBS_PER_GROUP;
  for (i = 0; i < EXITSTUBS_PER_GROUP; i++)
    *mxp++ = A64I_B|((-5-i)&0x03ffffffu);
  lj_mcode_sync(as->mcbot, mxp);
  lj_mcode_commitbot(as->J, mxp);
  as->mcbot = mxp;
  as->mclim = as->mcbot + MCLIM_REDZONE;
  return mxp - EXITSTUBS_PER_GROUP;
}

/* Setup all needed exit stubs. */
static void asm_exitstub_setup(ASMState *as, ExitNo nexits)
{
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
    emit_cond_branch(as, cc^1, p-1); /* branch to self, patched in asm_loop_fixup */
    return;
  }
  /* ARM64 doesn't have conditional BL, so we emit an unconditional BL
     and branch around it with the opposite condition */
  emit_branch(as, A64I_BL, target);
  emit_cond_branch(as, cc^1, p);
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
			  A64Ins ins)
{
  IRIns *ir = IR(ref);
  if (ra_noreg(ir->r)) {
    if (ir->o == IR_AREF) {
      if (mayfuse(as, ref)) {
        if (irref_isk(ir->op2)) {
          IRRef tab = IR(ir->op1)->op1;
          int32_t ofs = asm_fuseabase(as, tab);
          IRRef refa = ofs ? tab : ir->op1;
          ofs += 8*IR(ir->op2)->i;
          if (check_offset(ins, ofs + (ofs > 0 ? 4 : 0)) != OFS_INVALID) {
            *ofsp = ofs;
            return ra_alloc1(as, refa, allow);
          }
        }
      }
    } else if (ir->o == IR_HREFK) {
      if (mayfuse(as, ref)) {
        int32_t ofs = (int32_t)(IR(ir->op2)->op2 * sizeof(Node));
        if (check_offset(ins, ofs + (ofs > 0 ? 4 : 0)) != OFS_INVALID) {
          *ofsp = ofs;
          return ra_alloc1(as, ir->op1, allow);
        }
      }
    } else if (ir->o == IR_UREFC) {
      lua_unimpl(); // x86 has LJ_GC64 changes here
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
  } else if (irref_isk(ref) && !irt_is64(ir->t)) {
    uint32_t k = emit_isk12(ai, ir->i);
    if (k != -1)
      return k ^ A64I_BINOPk;
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
  uint32_t n, nargs = CCI_XNARGS(ci);
  int32_t ofs = 0;
  Reg gpr, fpr = REGARG_FIRSTFPR;

  if ((void *)ci->func) {
    emit_call(as, (void *)ci->func);
  }

  for (gpr = REGARG_FIRSTGPR; gpr <= REGARG_LASTGPR; gpr++)
    as->cost[gpr] = REGCOST(~0u, ASMREF_L);
  gpr = REGARG_FIRSTGPR;

  for (n = 0; n < nargs; n++) { /* Setup args. */
    IRRef ref = args[n];
    IRIns *ir = IR(ref);
    // TODO: check the case (ci->flags & CCI_VARARG)
    if (ref) {
      if (irt_isfp(ir->t)) {
        if (fpr <= REGARG_LASTFPR) {
          lua_assert(rset_test(as->freeset, fpr)); /* Must have been evicted. */
          ra_leftov(as, fpr, ref);
          fpr++;
        } else {
          Reg r = ra_alloc1(as, ref, RSET_FPR);
          emit_spstore(as, ir, r, ofs);
          ofs += 8;
        }
      } else {
        if (gpr <= REGARG_LASTGPR) {
          lua_assert(rset_test(as->freeset, gpr)); /* Must have been evicted. */
          ra_leftov(as, gpr, ref);
          gpr++;
        } else {
          Reg r = ra_alloc1(as, ref, RSET_GPR);
          emit_spstore(as, ir, r, ofs);
          ofs += 8;
        }
      }
    }
  }
}

/* Setup result reg/sp for call. Evict scratch regs. */
static void asm_setupresult(ASMState *as, IRIns *ir, const CCallInfo *ci)
{
  RegSet drop = RSET_SCRATCH;
  if (ra_hasreg(ir->r))
    rset_clear(drop, ir->r); /* Dest reg handled below. */
  ra_evictset(as, drop); /* Evictions must be performed first. */
  if (ra_used(ir)) {
    lua_assert(!irt_ispri(ir->t));
    if (irt_isfp(ir->t)) {
      ra_destreg(as, ir, RID_FPRET);
    } else {
      ra_destreg(as, ir, RID_RET);
    }
  }
  UNUSED(ci);
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
  RegSet allow = RSET_FPR;
  Reg left = ra_alloc1(as, ir->op1, allow);
  Reg right = ra_alloc1(as, ir->op2, rset_clear(allow, left));
  Reg tmp = ra_scratch(as, rset_clear(allow, right));
  Reg dest = ra_dest(as, ir, RSET_GPR);
  emit_dn(as, A64I_FMOV_R_S, dest, (tmp & 31));
  emit_dnm(as, A64I_FADDd, (tmp & 31), (left & 31), (right & 31));
}
#endif

static void asm_conv(ASMState *as, IRIns *ir)
{
  IRType st = (IRType)(ir->op2 & IRCONV_SRCMASK);
  int stfp = (st == IRT_NUM || st == IRT_FLOAT);
  IRRef lref = ir->op1;
  /* 64 bit integer conversions are handled by SPLIT. */
  /* TODO: 64-bit conversions should be handled here? */
  lua_assert(!irt_isint64(ir->t) && !(st == IRT_I64 || st == IRT_U64));
  lua_assert(irt_type(ir->t) != st);
  if (irt_isfp(ir->t)) {
    Reg dest = ra_dest(as, ir, RSET_FPR);
    if (stfp) {  /* FP to FP conversion. */
      emit_dm(as, st == IRT_NUM ? A64I_FCVT_F32_F64 : A64I_FCVT_F64_F32,
	      (dest & 31), (ra_alloc1(as, lref, RSET_FPR) & 31));
    } else {  /* Integer to FP conversion. */
      Reg left = ra_alloc1(as, lref, RSET_GPR);
      A64Ins ai = irt_isfloat(ir->t) ?
	(st == IRT_INT ? A64I_FCVT_F32_S32 : A64I_FCVT_F32_U32) :
	(st == IRT_INT ? A64I_FCVT_F64_S32 : A64I_FCVT_F64_U32);
      emit_dn(as, ai, (dest & 31), left);
    }
  } else if (stfp) {  /* FP to integer conversion. */
    if (irt_isguard(ir->t)) {
      /* Checked conversions are only supported from number to int. */
      lua_assert(irt_isint(ir->t) && st == IRT_NUM);
      asm_tointg(as, ir, ra_alloc1(as, lref, RSET_FPR));
    } else {
      Reg left = ra_alloc1(as, lref, RSET_FPR);
      Reg dest = ra_dest(as, ir, RSET_GPR);
      A64Ins ai;
      ai = irt_isint(ir->t) ?
	(st == IRT_NUM ? A64I_FCVT_S32_F64 : A64I_FCVT_S32_F32) :
	(st == IRT_NUM ? A64I_FCVT_U32_F64 : A64I_FCVT_U32_F32);
      emit_dm(as, ai, dest, (left & 31));
    }
  } else {
    Reg dest = ra_dest(as, ir, RSET_GPR);
    if (st >= IRT_I8 && st <= IRT_U16) {  /* Extend to 32 bit integer. */
      Reg left = ra_alloc1(as, lref, RSET_GPR);
      lua_assert(irt_isint(ir->t) || irt_isu32(ir->t));
      A64Ins ai = st == IRT_I8 ? A64I_SXTBw :
		  st == IRT_U8 ? A64I_UXTBw :
		  st == IRT_I16 ? A64I_SXTHw : A64I_UXTHw;
      emit_dm(as, ai, dest, left);
    } else {  /* Handle 32/32 bit no-op (cast). */
      ra_leftov(as, dest, lref);  /* Do nothing, but may need to move regs. */
    }
  }
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
  int32_t ofs = (int32_t)(kslot->op2 * sizeof(Node));
  int32_t kofs = ofs + (int32_t)offsetof(Node, key);
  int large_ofs = check_offset(A64I_LDRx, ofs) == OFS_INVALID;
  Reg dest = (ra_used(ir) || large_ofs) ? ra_dest(as, ir, RSET_GPR) : RID_NONE;
  Reg node = ra_alloc1(as, ir->op1, RSET_GPR);
  Reg key = RID_NONE, type = RID_TMP, idx = node;
  RegSet allow = rset_exclude(RSET_GPR, node);
  lua_assert(ofs % sizeof(Node) == 0);
  if (large_ofs) {
    idx = dest;
    rset_clear(allow, dest);
    kofs = (int32_t)offsetof(Node, key);
    lua_unimpl();
  } else if (ra_hasreg(dest)) {
    emit_opk(as, A64I_ADDx, dest, node, ofs, allow); /*!!!TODO w or x */
  }
  asm_guardcc(as, CC_NE);

  if (!irt_ispri(irkey->t)) {
    Reg key = ra_scratch(as, rset_exclude(RSET_GPR, node));
    emit_dnm(as, A64I_CMPx, 0, dest, key);
    lua_assert(irt_isnum(irkey->t) || irt_isgcv(irkey->t));
    /* !!!TODO is this valid on ARM64? */
    /* Assumes -0.0 is already canonicalized to +0.0. */
    emit_loadu64(as, key, irt_isnum(irkey->t) ? ir_knum(irkey)->u64 :
                          ((uint64_t)irt_toitype(irkey->t) << 47) |
                          (uint64_t)ir_kgc(irkey));
    emit_lso(as, A64I_LDRx, dest, idx, kofs);
  } else {
    lua_unimpl();
    lua_assert(!irt_isnil(irkey->t));
//    emit_i32(as, (irt_toitype(irkey->t)<<15)|0x7fff);
//    emit_rmro(as, XO_ARITHi, XOg_CMP, node,
//              ofs + (int32_t)offsetof(Node, key.it));
  }

  if (large_ofs)
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
  case IRT_I8: return A64I_LDRBw ^ A64I_LS_S;
  case IRT_U8: return A64I_LDRBw;
  case IRT_I16: return A64I_LDRHw ^ A64I_LS_S;
  case IRT_U16: return A64I_LDRHw;
  case IRT_NUM: return A64I_LDRd;
  case IRT_FLOAT: lua_unimpl(); return /*ARMI_VLDR_S*/0;
  case IRT_P64: return A64I_LDRx;
  case IRT_INT: return A64I_LDRw;
  case IRT_TAB: lua_todo(); return A64I_LDRx; // !!!TODO dunno yet!
  default: lua_unimpl(); return A64I_LDRx;
  }
}

static A64Ins asm_fxstoreins(IRIns *ir)
{
  lua_unimpl();
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
  Reg idx, tmp, type;
  int32_t ofs = 0;
  RegSet gpr = RSET_GPR, allow = irt_isnum(ir->t) ? RSET_FPR : RSET_GPR;
  lua_assert(irt_isnum(ir->t) || irt_ispri(ir->t) || irt_isaddr(ir->t) ||
	     (LJ_DUALNUM && irt_isint(ir->t)));
  if (ra_used(ir)) {
    Reg dest = ra_dest(as, ir, allow);
    gpr = rset_exclude(gpr, dest);
    tmp = irt_isnum(ir->t) ? ra_scratch(as, gpr) : dest;
    if (irt_isaddr(ir->t)) {
      /* TODO: Implement AND. LSL->LSR is suboptimal. */
      /* emit_opk(as, A64I_ANDx, dest, dest, LJ_GCVMASK, gpr); */
      lua_todo();
      emit_dn(as, A64I_LSRx|A64F_IR(17), dest, tmp);
      emit_dn(as, A64I_LSLx|A64F_IR((-17%64)&0x3f)|A64F_IS(63-17), tmp, tmp);
    } else if (irt_isnum(ir->t)) {
      emit_dn(as, A64I_FMOV_D_R, (dest & 31), tmp);
    }
  } else
    tmp = ra_scratch(as, gpr);
  gpr = rset_exclude(gpr, tmp);
  type = ra_scratch(as, gpr);
  gpr = rset_exclude(gpr, type);
  idx = asm_fuseahuref(as, ir->op1, &ofs, gpr, A64I_LDRx);
  gpr = rset_exclude(gpr, idx);
  /* Always do the type check, even if the load result is unused. */
  asm_guardcc(as, irt_isnum(ir->t) ? CC_LO : CC_NE);
  if (irt_type(ir->t) >= IRT_NUM) {
    lua_assert(irt_isinteger(ir->t) || irt_isnum(ir->t));
    /* TODO: Not tested. Is it better to allocate register or load constant
     * every time? TISNUM should be quite commonly used constant, so allocating
     * a register could be a good option. */
    /* Reg tisnum = ra_allock(as, LJ_TISNUM << 15, gpr); */
    lua_todo();
    emit_nm(as, A64I_CMPx|A64F_SH(A64SH_LSR, 32), type, tmp);
    emit_loadu64(as, type, LJ_TISNUM << 15);
  } else if (irt_isaddr(ir->t)) {
    emit_opk(as, A64I_CMNx, RID_ZERO, type, -irt_toitype(ir->t), gpr);
    emit_dn(as, A64I_ASRx|A64F_IR(47), type, tmp);
  } else if (irt_isnil(ir->t)) {
    /* TODO: Not tested. */
    lua_todo();
    emit_opk(as, A64I_CMNx, RID_ZERO, tmp, -1, gpr);
  } else {
    /* TODO: Not tested. */
    lua_todo();
    emit_nm(as, A64I_CMPx|A64F_SH(A64SH_LSR, 32), type, tmp);
    emit_loadu64(as, type, (irt_toitype(ir->t) << 15) | 0x7fff);
  }
  emit_lso(as, A64I_LDRx, tmp, idx, ofs);
}

static void asm_ahustore(ASMState *as, IRIns *ir)
{
  lua_todo(); /* !!!TODO x86 has loads of GC64 changes here */
  if (ir->r != RID_SINK) {
    RegSet allow = RSET_GPR;
    Reg idx, src = RID_NONE, type = RID_NONE;
    int32_t ofs = 0;
    if (irt_isnum(ir->t)) {
      src = ra_alloc1(as, ir->op2, RSET_FPR);
      idx = asm_fuseahuref(as, ir->op1, &ofs, allow, A64I_STRd);
      emit_lso(as, A64I_STRd, (src & 31), idx, ofs);
    } else {
      if (!irt_ispri(ir->t)) {
	src = ra_alloc1(as, ir->op2, allow);
	rset_clear(allow, src);
      }
      type = ra_allock(as, (int32_t)irt_toitype(ir->t)<<15, allow);
      idx = asm_fuseahuref(as, ir->op1, &ofs, rset_exclude(allow, type), A64I_STRw);
      if (ra_hasreg(src))
	emit_lso(as, A64I_STRw, src, idx, ofs); /* !!!TODO STRx? */
      emit_lso(as, A64I_STRw, type, idx, ofs+4); /* !!!TODO STRx? */
    }
  }
}

static void asm_sload(ASMState *as, IRIns *ir)
{
  int32_t ofs = 8*((int32_t)ir->op1-2) + ((ir->op2 & IRSLOAD_FRAME) ? 4 : 0);
  IRType t = irt_type(ir->t);
  Reg dest = RID_NONE, type = RID_NONE, base;
  RegSet allow = RSET_GPR;
  lua_assert(!(ir->op2 & IRSLOAD_PARENT));  /* Handled by asm_head_side(). */
  lua_assert(irt_isguard(ir->t) || !(ir->op2 & IRSLOAD_TYPECHECK));
  if ((ir->op2 & IRSLOAD_CONVERT) && irt_isguard(ir->t) && t == IRT_INT) {
    dest = ra_scratch(as, RSET_FPR);
    asm_tointg(as, ir, dest);
    t = IRT_NUM;  /* Continue with a regular number type check. */
  } else
  if (ra_used(ir)) {
    Reg tmp = RID_NONE;
    if ((ir->op2 & IRSLOAD_CONVERT))
      tmp = ra_scratch(as, t == IRT_INT ? RSET_FPR : RSET_GPR);
    lua_assert(irt_isnum(ir->t) || irt_isint(ir->t) || irt_isaddr(ir->t));
    dest = ra_dest(as, ir, (t == IRT_NUM) ? RSET_FPR : allow);
    rset_clear(allow, dest);
    base = ra_alloc1(as, REF_BASE, allow);
    if ((ir->op2 & IRSLOAD_CONVERT)) {
      if (t == IRT_INT) {
        lua_unimpl();
        //emit_dn(as, ARMI_VMOV_R_S, dest, (tmp & 15));
        //emit_dm(as, ARMI_VCVT_S32_F64, (tmp & 15), (tmp & 15));
        t = IRT_NUM;  /* Check for original type. */
      } else {
        lua_unimpl();
        //emit_dm(as, ARMI_VCVT_F64_S32, (dest & 15), (dest & 15));
        //emit_dn(as, ARMI_VMOV_S_R, tmp, (dest & 15));
        t = IRT_INT;  /* Check for original type. */
      }
      dest = tmp;
    }
    goto dotypecheck;
  }
  base = ra_alloc1(as, REF_BASE, allow);
dotypecheck:
  rset_clear(allow, base);
  if ((ir->op2 & IRSLOAD_TYPECHECK)) {
    if (ra_noreg(type)) {
      if (ofs < 256 && ra_hasreg(dest) && (dest & 1) == 0 &&
          rset_test((as->freeset & allow), dest+1)) {
        type = dest+1;
        ra_modified(as, type);
      } else {
        type = RID_TMP;
      }
    }
    asm_guardcc(as, t == IRT_NUM ? CC_HS : CC_NE);
    emit_opk(as, A64I_CMNx, 0, type, -irt_toitype(ir->t), allow);
  }

  if (ra_hasreg(type)) {
    emit_dn(as, A64I_ASRx|A64F_IR(47), type, dest);
  }

  if (ra_hasreg(dest)) {
    if (t == IRT_NUM) {
      if (check_offset(A64I_LDRd, ofs) != OFS_INVALID) {
        emit_lso(as, A64I_LDRd, dest, base, ofs);
      } else {
        /* !!!TODO w or x */
        if (ra_hasreg(type)) emit_lso(as, A64I_LDRw, type, RID_TMP, 4);
        emit_lso(as, A64I_LDRd, dest, RID_TMP, 0);
        emit_opk(as, A64I_ADDx, RID_TMP, base, ofs, allow);
        return;
      }
    } else
      emit_lso(as, A64I_LDRx, dest, base, ofs); /* !!!!TODO w or x */
  }
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

static void asm_fparith(ASMState *as, IRIns *ir, A64Ins ai)
{
  Reg dest = ra_dest(as, ir, RSET_FPR);
  Reg right, left = ra_alloc2(as, ir, RSET_FPR);
  right = (left >> 8); left &= 255;
  emit_dnm(as, ai, (dest & 31), (left & 31), (right & 31));
}

static void asm_fpunary(ASMState *as, IRIns *ir, A64Ins ai)
{
  Reg dest = ra_dest(as, ir, RSET_FPR);
  Reg left = ra_hintalloc(as, ir->op1, dest, RSET_FPR);
  emit_dn(as, ai, (dest & 31), (left & 31));
}

static void asm_callround(ASMState *as, IRIns *ir, int id)
{
    lua_unimpl();
}

static void asm_fpmath(ASMState *as, IRIns *ir)
{
    lua_unimpl();
}

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
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_hintalloc(as, ir->op1, dest, RSET_GPR);
  emit_dm(as, ai, dest, left);
}

/* NYI: use add/shift for MUL(OV) with constants. FOLD only does 2^k. */
static void asm_intmul(ASMState *as, IRIns *ir)
{
  Reg dest = ra_dest(as, ir, RSET_GPR);
  Reg left = ra_alloc1(as, ir->op1, rset_exclude(RSET_GPR, dest));
  Reg right = ra_alloc1(as, ir->op2, rset_exclude(RSET_GPR, left));
  if (irt_isguard(ir->t)) {  /* IR_MULOV */
    asm_guardcc(as, CC_NE);
    emit_nm(as, A64I_CMPw|A64F_SH(A64SH_ASR, 31), RID_TMP, dest);
    emit_dn(as, A64I_ASRx|A64F_IR(32), RID_TMP, dest);
    emit_dnm(as, A64I_SMULL, dest, right, left);
  } else {
    emit_nm(as, A64I_MULx, dest, left);
  }
}

static void asm_add(ASMState *as, IRIns *ir)
{
  if (irt_isnum(ir->t)) {
    if (!asm_fusemadd(as, ir, A64I_FMADDd, A64I_FMADDd))
      asm_fparith(as, ir, A64I_FADDd);
    return;
  }
  asm_intop_s(as, ir, A64I_ADDw); /* !!!TODO this drops the tag, is it wrong? */
}

static void asm_sub(ASMState *as, IRIns *ir)
{
  if (irt_isnum(ir->t)) {
    asm_fparith(as, ir, A64I_FSUBd);
    return;
  }
  asm_intop_s(as, ir, A64I_SUBw); /* !!!TODO this drops the tag, is it wrong? */
}

static void asm_mul(ASMState *as, IRIns *ir)
{
  if (irt_isnum(ir->t)) {
    asm_fparith(as, ir, A64I_FMULd);
    return;
  }
  asm_intmul(as, ir);
}

#define asm_addov(as, ir)	asm_add(as, ir)
#define asm_subov(as, ir)	asm_sub(as, ir)
#define asm_mulov(as, ir)	asm_mul(as, ir)

#define asm_div(as, ir)		asm_fparith(as, ir, A64I_FDIVd)
#define asm_pow(as, ir)		asm_callid(as, ir, IRCALL_lj_vm_powi)
#define asm_abs(as, ir)		asm_fpunary(as, ir, A64I_FABS)
#define asm_atan2(as, ir)	asm_callid(as, ir, IRCALL_atan2)
#define asm_ldexp(as, ir)	asm_callid(as, ir, IRCALL_ldexp)

#define asm_mod(as, ir)		asm_callid(as, ir, IRCALL_lj_vm_modi)

static void asm_neg(ASMState *as, IRIns *ir)
{
#if !LJ_SOFTFP
  if (irt_isnum(ir->t)) {
    asm_fpunary(as, ir, A64I_FNEGd);
    return;
  }
#endif
  asm_intneg(as, ir, A64I_NEGx);
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
  Reg left, right;
  A64Ins ai;
  int swp = ((ir->o ^ (ir->o >> 2)) & ~(ir->o >> 3) & 1);
  if (!swp && irref_isk(ir->op2) && ir_knum(IR(ir->op2))->u64 == 0) {
    left = (ra_alloc1(as, ir->op1, RSET_FPR) & 31);
    right = 0;
    ai = A64I_FCMPZd;
  } else {
    left = ra_alloc2(as, ir, RSET_FPR);
    if (swp) {
      right = (left & 31); left = ((left >> 8) & 31);
    } else {
      right = ((left >> 8) & 31); left &= 31;
    }
    ai = A64I_FCMPd;
  }
  asm_guardcc(as, (asm_compmap[ir->o] >> 4));
  emit_nm(as, ai, left, right);
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
  SnapEntry *map = &as->T->snapmap[snap->mapofs];
  SnapEntry *flinks = &as->T->snapmap[snap_nextofs(as->T, snap)-1-LJ_FR2];
  MSize n, nent = snap->nent;
  /* Store the value of all modified slots to the Lua stack. */
  for (n = 0; n < nent; n++) {
    SnapEntry sn = map[n];
    BCReg s = snap_slot(sn);
    int32_t ofs = 8*((int32_t)s-1);
    IRRef ref = snap_ref(sn);
    IRIns *ir = IR(ref);
    if ((sn & SNAP_NORESTORE))
      continue;
    if (irt_isnum(ir->t)) {
      Reg src = ra_alloc1(as, ref, RSET_FPR);
      emit_lso(as, A64I_STRd, src, RID_BASE, ofs);
    } else {
      lua_assert(irt_ispri(ir->t) || irt_isaddr(ir->t) ||
                 (LJ_DUALNUM && irt_isinteger(ir->t)));
      if (!irref_isk(ref)) {
        Reg src = ra_alloc1(as, ref, rset_exclude(RSET_GPR, RID_BASE));
        if (irt_is64(ir->t)) {
          /* TODO: 64 bit store + 32 bit load-modify-store is suboptimal. */
          emit_lso(as, A64I_STRw, RID_TMP, RID_BASE, ofs+4);
          emit_opk(as, A64I_ORRw, 0, RID_TMP, -(irt_toitype(ir->t)<<15), RSET_GPR);
          emit_lso(as, A64I_LDRw, RID_TMP, RID_BASE, ofs+4);

          //emit_u32(as, irt_toitype(ir->t) << 15);
          //emit_rmro(as, XO_ARITHi, XOg_OR, RID_BASE, ofs+4);
        } else if (LJ_DUALNUM && irt_isinteger(ir->t)) {
          emit_lso(as, A64I_STRw, RID_TMP, RID_BASE, ofs+4);
          emit_loadi(as, RID_TMP, LJ_TISNUM << 15);

          //emit_movmroi(as, RID_BASE, ofs+4, LJ_TISNUM << 15);
        } else {
          emit_lso(as, A64I_STRw, RID_TMP, RID_BASE, ofs+4);
          emit_loadi(as, RID_TMP, (irt_toitype(ir->t)<<15)|0x7fff);

          //emit_movmroi(as, RID_BASE, ofs+4, (irt_toitype(ir->t)<<15)|0x7fff);
        }
        if (irt_is64(ir->t))
        {
          emit_lso(as, A64I_STRx, src, RID_BASE, ofs);
          //emit_movtomro(as, REX_64IR(ir, src), RID_BASE, ofs);
        } else {
          emit_lso(as, A64I_STRw, src, RID_BASE, ofs);
          //emit_movtomro(as, REX_64IR(ir, src), RID_BASE, ofs);
        }
      } else {
        TValue k;
        lj_ir_kvalue(as->J->L, &k, ir);
        if (tvisnil(&k)) {
          emit_lso(as, A64I_STRx, RID_TMP, RID_BASE, ofs);
          // !!!TODO (uint32_t)-1 or (uint64_t)-1
          emit_loadu64(as, RID_TMP, 0xffffffff);
          //emit_i32(as, -1);
          //emit_rmro(as, XO_MOVmi, REX_64, RID_BASE, ofs);
        } else {
          emit_lso(as, A64I_STRx, RID_TMP, RID_BASE, ofs);
          emit_loadu64(as, RID_TMP, k.u64);
          //emit_movmroi(as, RID_BASE, ofs+4, k.u32.hi);
          //emit_movmroi(as, RID_BASE, ofs, k.u32.lo);
        }
      }
    }
    checkmclim(as);
  }
  lua_assert(map + nent == flinks);
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
    ptrdiff_t delta = target - (p - 2);
    lua_assert(((delta + 0x40000) >> 19) == 0);
    /* asm_guardcc already inverted the bcc and patched the final bl. */
    p[-2] |= ((uint32_t)delta & 0x7ffff) << 5;
  } else {
    lua_unimpl(); /* what's going on here? */
    p[-1] = A64I_B | ((uint32_t)((target-p)+1) & 0x03ffffffu);
  }
}

/* -- Head of trace ------------------------------------------------------- */

/* Reload L register from g->cur_L. */
static void asm_head_lreg(ASMState *as)
{
  IRIns *ir = IR(ASMREF_L);
  if (ra_used(ir)) {
    Reg r = ra_dest(as, ir, RSET_GPR);
    emit_getgl(as, r, cur_L);
    ra_evictk(as);
  }
}

/* Coalesce BASE register for a root trace. */
static void asm_head_root_base(ASMState *as)
{
  IRIns *ir;
  asm_head_lreg(as);
  ir = IR(REF_BASE);
  if (ra_hasreg(ir->r) && (rset_test(as->modset, ir->r) || irt_ismarked(ir->t)))
    ra_spill(as, ir);
  ra_destreg(as, ir, RID_BASE);
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
  IRRef args[CCI_NARGS_MAX * 2];
  uint32_t i, nargs = CCI_XNARGS(ci);
  int nslots = 0, ngpr = REGARG_NUMGPR, nfpr = REGARG_NUMFPR;
  asm_collectargs(as, ir, ci, args);
  for (i = 0; i < nargs; i++) {
    // TODO: check the case (ci->flags & CCI_VARARG)
    if (args[i] && irt_isfp(IR(args[i])->t)) {
      if (nfpr > 0) {
        nfpr--;
      } else {
        nslots++;
      }
    } else {
      if (ngpr > 0) {
        ngpr--;
      } else {
        nslots++;
      }
    }
  }
  if (nslots > as->evenspill) {
    as->evenspill = nslots;
  }
  // TODO : Return REGSP_HINT(RID_FPRET) for floating numbers?
  return REGSP_HINT(RID_RET);
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

