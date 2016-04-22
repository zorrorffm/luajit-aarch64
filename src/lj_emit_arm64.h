/*
** ARM64 instruction emitter.
** Copyright !!!TODO
*/

/* Load a 32 bit constant into a GPR. */
static void emit_loadi(ASMState *as, Reg r, int32_t i)
{
  lua_unimpl();
}

static void emit_loadn(ASMState *as, Reg r, cTValue *tv)
{
  lua_unimpl();
}

/* !!!TODO non-ARM ports are different */
#define emit_canremat(ref)      ((ref) < ASMREF_L)

#define emit_getgl(as, r, field) lua_unimpl()

/* mov r, imm64 or shorter 32 bit extended load. */
static void emit_loadu64(ASMState *as, Reg r, uint64_t u64)
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

/* Add offset to pointer. */
static void emit_addptr(ASMState *as, Reg r, int32_t ofs)
{
  lua_unimpl();
}

#define emit_jmp(as, target) lua_unimpl()

#define emit_setvmstate(as, i)                UNUSED(i)
#define emit_spsub(as, i)                     lua_unimpl()
#define emit_setgl(as, r, field)              lua_unimpl()


