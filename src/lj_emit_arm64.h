/*
** ARM64 instruction emitter.
** Copyright !!!TODO
*/

static Reg ra_allock(ASMState *as, int32_t k, RegSet allow);

static void emit_loadn(ASMState *as, Reg r, cTValue *tv)
{
  lua_unimpl();
}

/* !!!TODO non-ARM ports are different */
#define emit_canremat(ref)      ((ref) < ASMREF_L)

#define emit_getgl(as, r, field) \
  emit_lsptr(as, A64I_LDRx, (r), (void *)&J2G(as->J)->field)


/* Encode constant in K12 format for data processing instructions. */
static uint32_t emit_isk12(A64Ins ai, int32_t n)
{
    /* !!!TODO implement this! */
    return -1;
}

/* Load a 32 bit constant into a GPR. */
static void emit_loadi(ASMState *as, Reg rd, int32_t i)
{
  if (i >= 0x10000) {
    *--as->mcp = A64I_MOVK_16w | A64F_D(rd) | A64F_U16((i>>16) & 0xffff);
  }
  *--as->mcp = A64I_MOVZw | A64F_D(rd) | A64F_U16(i & 0xffff);
}

/* mov r, imm64 or shorter 32 bit extended load. */
static void emit_loadu64(ASMState *as, Reg r, uint64_t u64)
{
  lua_unimpl();
}

/* Load a number constant into an FPR. */
/* !!!TODO maybe also GPR on arm64? */
static void emit_loadk64(ASMState *as, Reg r, IRIns *ir)
{
  lua_unimpl();
}


/* Generic load of register with base and (small) offset address. */
static void emit_loadofs(ASMState *as, IRIns *ir, Reg r, Reg base, int32_t ofs)
{
  lua_unimpl();
}

/* Generic store of register with base and (small) offset address. */
static void emit_storeofs(ASMState *as, IRIns *ir, Reg r, Reg base, int32_t ofs)
{
  lua_unimpl();
}

/* Generic move between two regs. */
static void emit_movrr(ASMState *as, IRIns *ir, Reg dst, Reg src)
{
  lua_unimpl();
}

static void emit_n(ASMState *as, A64Ins ai, Reg rn)
{
  *--as->mcp = ai | A64F_N(rn);
}

static void emit_dn(ASMState *as, A64Ins ai, Reg rd, Reg rn)
{
  *--as->mcp = ai | A64F_D(rd) | A64F_N(rn);
}

static void emit_dnm(ASMState *as, A64Ins ai, Reg rd, Reg rn, Reg rm)
{
  *--as->mcp = ai | A64F_D(rd) | A64F_N(rn) | A64F_M(rm);
}

/* Emit an arithmetic/logic operation with a constant operand. */
static void emit_opk(ASMState *as, A64Ins ai, Reg dest, Reg src,
                     int32_t i, RegSet allow)
{
  uint32_t k = emit_isk12(ai, i);
  if (k != -1)
    emit_dn(as, ai^k, dest, src);
  else
    emit_dnm(as, ai, dest, src, ra_allock(as, i, allow));
}

static void emit_ccmpr(ASMState *as, A64Ins ai, A64CC cond, int32_t nzcv, Reg
rn, Reg rm)
{
  *--as->mcp = ai | A64F_N(rn) | A64F_M(rm) | A64F_NZCV(nzcv) | A64F_COND(cond);
}

static void emit_ccmpk(ASMState *as, A64Ins ai, A64CC cond, int32_t nzcv, Reg
rn, int32_t k, RegSet allow)
{
  if (k >=0 && k <= 31)
    *--as->mcp =
      ai | A64F_N (rn) | A64F_M (k) | A64F_NZCV (nzcv) | A64F_COND (cond);
  else
  {
    emit_ccmpr(as, ai, cond, nzcv, rn, ra_allock(as, k, allow));
  }
}
 
/* -- Emit loads/stores --------------------------------------------------- */

static void emit_lso(ASMState *as, A64Ins ai, Reg rd, Reg rn, int32_t ofs)
{
  /* !!!TODO ARM emit_lso combines LDR/STR pairs into LDRD/STRD, something
     similar possible here? */
  /* !!!TODO support STUR encodings, these ranges don't match emit_arm64... */
  lua_assert(ofs >= 0 && ofs <= 4096 && (ofs&3) == 0);
  //if (ofs < 0) ofs = -ofs; else ai |= ARMI_LS_U;
  *--as->mcp = ai | A64F_D(rd) | A64F_N(rn) | A64F_A(ofs >> 3);
}

static void emit_lsptr(ASMState *as, A64Ins ai, Reg r, void *p)
{
  int64_t i = i64ptr(p);
  Reg tmp = RID_TMP; /*!!!TODO allocate register? */
  emit_lso(as, ai, r, tmp, (i & 0xffff));
  *--as->mcp = A64I_MOVK_48x | A64F_D(tmp) | A64F_U16((i>>48) & 0xffff);
  *--as->mcp = A64I_MOVK_32x | A64F_D(tmp) | A64F_U16((i>>32) & 0xffff);
  *--as->mcp = A64I_MOVK_16x | A64F_D(tmp) | A64F_U16((i>>16) & 0xffff);

}

/* -- Emit control-flow instructions -------------------------------------- */

static void emit_branch(ASMState *as, A64Ins ai, MCode *target)
{
  MCode *p = as->mcp;
  ptrdiff_t delta = target - p;
  lua_assert(((delta + 0x02000000) >> 26) == 0);
  *--p = ai | ((uint32_t)delta & 0x00ffffffu);
  as->mcp = p;
}

static void emit_cond_branch(ASMState *as, A64CC cond, MCode *target)
{
  MCode *p = as->mcp;
  ptrdiff_t delta = target - p;
  lua_assert(((delta + 0x40000) >> 19) == 0);
  *--p = A64I_Bcond | (((uint32_t)delta & 0x7ffff)<<5) | cond;
  as->mcp = p;
}

/* Add offset to pointer. */
static void emit_addptr(ASMState *as, Reg r, int32_t ofs)
{
  if (ofs)
  {
    A64Ins op = ofs < 0 ? A64I_SUBx : A64I_ADDx;
    if (ofs < 0)
      ofs = -ofs;
    emit_opk(as, op, r, r, ofs, rset_exclude(RSET_GPR, r));
  }
}

#define emit_jmp(as, target) lua_unimpl()

#define emit_setvmstate(as, i)                UNUSED(i)
#define emit_spsub(as, ofs)                   emit_addptr(as, RID_SP, -(ofs))
#define emit_setgl(as, r, field)              lua_unimpl()


