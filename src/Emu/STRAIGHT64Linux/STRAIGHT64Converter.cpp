// 
// Copyright (c) 2005-2008 Kenichi Watanabe.
// Copyright (c) 2005-2008 Yasuhiro Watari.
// Copyright (c) 2005-2008 Hironori Ichibayashi.
// Copyright (c) 2008-2009 Kazuo Horio.
// Copyright (c) 2009-2015 Naruki Kurata.
// Copyright (c) 2005-2015 Masahiro Goshima.
// Copyright (c) 2005-2017 Ryota Shioya.
// 
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 
// 1. The origin of this software must not be misrepresented; you must not
// claim that you wrote the original software. If you use this software
// in a product, an acknowledgment in the product documentation would be
// appreciated but is not required.
// 
// 2. Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original software.
// 
// 3. This notice may not be removed or altered from any source
// distribution.
// 
// 


#include <pch.h>

#include "Emu/STRAIGHT64Linux/STRAIGHT64Converter.h"

//#include "Emu/Utility/GenericOperation.h"
#include "Emu/Utility/OpEmulationState.h"

#include "Emu/STRAIGHT64Linux/STRAIGHT64Info.h"
#include "Emu/STRAIGHT64Linux/STRAIGHT64OpInfo.h"
#include "Emu/STRAIGHT64Linux/STRAIGHT64Decoder.h"
#include "Emu/STRAIGHT64Linux/STRAIGHT64Operation.h"
#include "Emu/STRAIGHT64Linux/STRAIGHT64Converter.h"
#include "Emu/RISCV32Linux/RISCV32Operation.h"

using namespace std;
using namespace boost;
using namespace Onikiri;
using namespace Onikiri::EmulatorUtility;
using namespace Onikiri::STRAIGHT64Linux;
using namespace Onikiri::EmulatorUtility::Operation;
using namespace Onikiri::STRAIGHT64Linux::Operation;
using namespace Onikiri::RISCV32Linux::Operation;

namespace {
    // オペランドのテンプレート
    // [RegTemplateBegin, RegTemplateEnd] は，命令中のオペランドレジスタ番号に置き換えられる
    // [ImmTemplateBegin, RegTemplateEnd] は，即値に置き換えられる

    // レジスタ・テンプレートに使用する番号
    // 本物のレジスタ番号を使ってはならない
    const int RegTemplateBegin = -20;    // 命令中のレジスタ番号に変換 (数値に意味はない．本物のレジスタ番号と重ならずかつ一意であればよい)
    const int RegTemplateEnd = RegTemplateBegin + 4 - 1;
    const int ImmTemplateBegin = -30;    // 即値に変換
    const int ImmTemplateEnd = ImmTemplateBegin + 2 - 1;

    const int R0 = RegTemplateBegin + 0;
    const int R1 = RegTemplateBegin + 1;
    const int R2 = RegTemplateBegin + 2;
    const int R3 = RegTemplateBegin + 3;
    const int R4 = RegTemplateBegin + 4;

    const int I0 = ImmTemplateBegin + 0;
}


//
// 命令の定義
//

namespace {
    // 各命令形式に対するオペコードを得るためのマスク (0のビットが引数)
    const u32 MASK_EXACT = 0xffffffff;  // 全bitが一致

    const u32 MASK_STB = 0x3f; // Store/Branchは最下位6bitがOPCODE
    const u32 MASK_ONEREG = 0x1ff; // 最下位13bitを見る
    const u32 MASK_SFTIMM = 0x183ffff;
    const u32 MASK_TWOREG = 0x3ffff; // 最下位18bit
}

#define STRAIGHT64_DSTOP(n) STRAIGHT64DstOperand<n>
#define STRAIGHT64_SRCOP(n) STRAIGHT64SrcOperand<n>

#define OPCODE_ONEREG32(opcode) (((opcode)<<8) | 0b10'000'0'1001111)
#define OPCODE_ONEREG64(opcode) (((opcode)<<8) | 0b10'000'1'1001111)

                                   // ..' imm ' x   '..'opc'.'.......
#define OPCODE_SFTIMM32(opcode, a) (0b00'00000'00000'10'000'0'1001111 | ((opcode)<<8) | ((a)<<17))
#define OPCODE_SFTIMM64(opcode, a) (0b00'00000'00000'10'000'1'1001111 | ((opcode)<<8) | ((a)<<17))

#define OPCODE_TWOREG32(op1, op2) (0b00000'11'000'0'1001111 | (op1 << 13) | (op2 << 8))
#define OPCODE_TWOREG64(op1, op2) (0b00000'11'000'1'1001111 | (op1 << 13) | (op2 << 8))


#define D0 STRAIGHT64_DSTOP(0)
#define D1 STRAIGHT64_DSTOP(1)
#define S0 STRAIGHT64_SRCOP(0)
#define S1 STRAIGHT64_SRCOP(1)
#define S2 STRAIGHT64_SRCOP(2)
#define S3 STRAIGHT64_SRCOP(3)

// 投機的にフェッチされたときにはエラーにせず，実行されたときにエラーにする
// syscallにすることにより，直前までの命令が完了してから実行される (実行は投機的でない)
STRAIGHT64Converter::OpDef STRAIGHT64Converter::m_OpDefUnknown = 
    {"unknown", MASK_EXACT, 0,  1, {{OpClassCode::UNDEF,    {-1, -1}, {I0, -1, -1}, STRAIGHTUnknownOperation}}};


// branchは，OpInfo 列の最後じゃないとだめ
STRAIGHT64Converter::OpDef STRAIGHT64Converter::m_OpDefsBase[] =
{
    //{Name,    Mask,        Opcode,        nOp, { OpClassCode,          Dst[],      Src[],          OpInfoType::EmulationFunc}[]}
    {"ST.8",    MASK_STB,    0b000111,      1,   { OpClassCode::iST,     {R0, -1},   {R1, I0, R2},   Set<D0, STRAIGHT64Store<u8, S0, STRAIGHT64Addr<S1, S2> > > }},
    {"ST.16",   MASK_STB,    0b100111,      1,   { OpClassCode::iST,     {R0, -1},   {R1, I0, R2},   Set<D0, STRAIGHT64Store<u16, S0, STRAIGHT64Addr<S1, S2> > > }},
    {"ST.32",   MASK_STB,    0b010111,      1,   { OpClassCode::iST,     {R0, -1},   {R1, I0, R2},   Set<D0, STRAIGHT64Store<u32, S0, STRAIGHT64Addr<S1, S2> > > }},
    {"ST.64",   MASK_STB,    0b110111,      1,   { OpClassCode::iST,     {R0, -1},   {R1, I0, R2},   Set<D0, STRAIGHT64Store<u64, S0, STRAIGHT64Addr<S1, S2> > > }},
    {"Blt",     MASK_STB,    0b000011,      1,   { OpClassCode::iBC,     {R0, -1},   {R1, R2, I0},   RISCV32BranchRelCond<S2, Compare<S0, S1, IntCondLessSigned<u32> > >}},
    {"Bge",     MASK_STB,    0b100011,      1,   { OpClassCode::iBC,     {R0, -1},   {R1, R2, I0},   RISCV32BranchRelCond<S2, Compare<S0, S1, IntCondGreaterEqualSigned<u32> > >}},
    {"Bltu",    MASK_STB,    0b010011,      1,   { OpClassCode::iBC,     {R0, -1},   {R1, R2, I0},   RISCV32BranchRelCond<S2, Compare<S0, S1, IntCondLessUnsigned<u32> > >}},
    {"Bgeu",    MASK_STB,    0b110011,      1,   { OpClassCode::iBC,     {R0, -1},   {R1, R2, I0},   RISCV32BranchRelCond<S2, Compare<S0, S1, IntCondGreaterEqualUnsigned<u32> > >}},
    {"Beq",     MASK_STB,    0b001011,      1,   { OpClassCode::iBC,     {R0, -1},   {R1, R2, I0},   RISCV32BranchRelCond<S2, Compare<S0, S1, IntCondEqual<u32> > >}},
    {"Bne",     MASK_STB,    0b101011,      1,   { OpClassCode::iBC,     {R0, -1},   {R1, R2, I0},   RISCV32BranchRelCond<S2, Compare<S0, S1, IntCondNotEqual<u32> > >}},

    // OneReg
    {"ADDi.32",  MASK_ONEREG, OPCODE_ONEREG32(0b000), 1, { OpClassCode::iALU, {R0, -1}, {R1, I0, -1}, Set<D0, IntAdd<u32, S0, S1> >}},
    {"SLTi.32",  MASK_ONEREG, OPCODE_ONEREG32(0b010), 1, { OpClassCode::iALU, {R0, -1}, {R1, I0, -1}, Set<D0, RISCV32Compare<S0, S1, IntCondLessSigned<u32> > >}},
    {"SLTiu.32", MASK_ONEREG, OPCODE_ONEREG32(0b011), 1, { OpClassCode::iALU, {R0, -1}, {R1, I0, -1}, Set<D0, RISCV32Compare<S0, S1, IntCondLessUnsigned<u32> > >}},
    {"XORi.32",  MASK_ONEREG, OPCODE_ONEREG32(0b100), 1, { OpClassCode::iALU, {R0, -1}, {R1, I0, -1}, Set<D0, BitXor<u32, S0, S1> >}},
    {"ORi.32",   MASK_ONEREG, OPCODE_ONEREG32(0b110), 1, { OpClassCode::iALU, {R0, -1}, {R1, I0, -1}, Set<D0, BitOr<u32, S0, S1> >}},
    {"ANDi.32",  MASK_ONEREG, OPCODE_ONEREG32(0b111), 1, { OpClassCode::iALU, {R0, -1}, {R1, I0, -1}, Set<D0, BitAnd<u32, S0, S1> >}},
    {"SLLi.32",  MASK_SFTIMM, OPCODE_SFTIMM32(0b001, 0), 1, { OpClassCode::iSFT, {R0, -1}, {R1, I0, -1}, Set<D0, LShiftL<u32, S0, S1, 0x1f> >}},
    {"SRLi.32",  MASK_SFTIMM, OPCODE_ONEREG32(0b101, 0), 1, { OpClassCode::iSFT, {R0, -1}, {R1, I0, -1}, Set<D0, LShiftR<u32, S0, S1, 0x1f> >}},
    {"SRAi.32",  MASK_SFTIMM, OPCODE_ONEREG32(0b101, 1), 1, { OpClassCode::iSFT, {R0, -1}, {R1, I0, -1}, Set<D0, AShiftR<u32, S0, S1, 0x1f> >}},

    // TwoReg
    {"ADD.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b000), 1, { OpClassCode::iALU, {R0, -1}, {R1, R2, -1}, Set<D0, IntAdd<u32, S0, S1> >}},
    {"SUB.32",   MASK_TWOREG, OPCODE_TWOREG32(0b01000, 0b000), 1, { OpClassCode::iALU, {R0, -1}, {R1, R2, -1}, Set<D0, IntSub<u32, S0, S1> >}},
    {"SLL.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b001), 1, { OpClassCode::iSFT, {R0, -1}, {R1, R2, -1}, Set<D0, LShiftL<u32, S0, S1, 0x1f> >}},
    {"SLT.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b010), 1, { OpClassCode::iALU, {R0, -1}, {R1, R2, -1}, Set<D0, Compare<S0, S1, IntCondLessSigned<u32> > >}},
    {"SLTu.32",  MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b011), 1, { OpClassCode::iALU, {R0, -1}, {R1, R2, -1}, Set<D0, Compare<S0, S1, IntCondLessUnsigned<u32> > >}},
    {"XOR.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b100), 1, { OpClassCode::iALU, {R0, -1}, {R1, R2, -1}, Set<D0, BitXor<u32, S0, S1> >}},
    {"SRL.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b101), 1, { OpClassCode::iSFT, {R0, -1}, {R1, R2, -1}, Set<D0, LShiftR<u32, S0, S1, 0x1f> >}},
    {"SRA.32",   MASK_TWOREG, OPCODE_TWOREG32(0b01000, 0b101), 1, { OpClassCode::iSFT, {R0, -1}, {R1, R2, -1}, Set<D0, AShiftR<u32, S0, S1, 0x1f> >}},
    {"OR.32",    MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b110), 1, { OpClassCode::iALU, {R0, -1}, {R1, R2, -1}, Set<D0, BitAnd<u32, S0, S1> >}},
    {"AND.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00000, 0b111), 1, { OpClassCode::iALU, {R0, -1}, {R1, R2, -1}, Set<D0, BitOr<u32, S0, S1> >}},
    {"MUL.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00001, 0b000), 1, { OpClassCode::iMUL, {R0, -1}, {R1, R2, -1}, Set<D0, IntMul<u32, S0, S1> >}},
    {"DIV.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00001, 0b100), 1, { OpClassCode::iDIV, {R0, -1}, {R1, R2, -1}, Set<D0, RISCV32IntDiv<S0, S1> >}},
    {"DIVu.32",  MASK_TWOREG, OPCODE_TWOREG32(0b00001, 0b101), 1, { OpClassCode::iDIV, {R0, -1}, {R1, R2, -1}, Set<D0, RISCV32IntDiv<S0, S1> >}},
    {"REM.32",   MASK_TWOREG, OPCODE_TWOREG32(0b00001, 0b110), 1, { OpClassCode::iDIV, {R0, -1}, {R1, R2, -1}, Set<D0, RISCV32IntRem<S0, S1> >}},
    {"REMu.32",  MASK_TWOREG, OPCODE_TWOREG32(0b00001, 0b111), 1, { OpClassCode::iDIV, {R0, -1}, {R1, R2, -1}, Set<D0, RISCV32IntRemu<S0, S1> >}},



    //{ "NOP",    MASK_OPCODE,  OPCODE(0),     1,   { { OpClassCode::iNOP,  { -1, -1 }, { -1, -1, -1 }, NoOperation } } },
    //{ "SYSCALL",MASK_OPCODE, OPCODE(1),     2,   {
    //    { OpClassCode::syscall,        { R0, -1 }, { I0,  1,  2 }, STRAIGHT64SyscallSetArg },
    //    { OpClassCode::syscall_branch, { R0, -1 }, { R0,  3,  4 }, STRAIGHT64SyscallCore },
    //} },
    //{ "J",      MASK_OPCODE, OPCODE(3),     1,  { { OpClassCode::iJUMP,  { R0, -1 }, { I0, -1, -1 }, Sequence2<BranchRelUncond<S0>, Set<D0, IntConst<u64, 0> > > } } },
    //{ "JR",     MASK_OPCODE, OPCODE(4),     1,  { { OpClassCode::iJUMP,  { R0, -1 }, { R1, -1, -1 }, Sequence2<BranchAbsUncond<S0>, Set<D0, IntConst<u64, 0> > > } } },
    //{ "JAL",    MASK_OPCODE, OPCODE(5),     1,  { { OpClassCode::CALL,   { R0, -1 }, { I0, -1, -1 }, CallRelUncond<D0, S0> } } },
    //{ "JRAL",   MASK_OPCODE, OPCODE(6),     1,  { { OpClassCode::CALL_JUMP,  { R0, -1 }, { R1, -1, -1 }, CallAbsUncond<D0, S0> } } },
    //{ "BEZ",    MASK_OPCODE, OPCODE(7),     1,  { { OpClassCode::iBC,    { R0, -1 }, { R1, I0, -1 }, Sequence2<BranchRelCond< S1, Compare< S0, IntConst<u64, 0>, IntCondEqual<u64> > >, Set<D0, IntConst<u64, 0> > > } } },
    //{ "BNZ",    MASK_OPCODE, OPCODE(8),     1,  { { OpClassCode::iBC,    { R0, -1 }, { R1, I0, -1 }, Sequence2<BranchRelCond< S1, Compare< S0, IntConst<u64, 0>, IntCondNotEqual<u64> > >, Set<D0, IntConst<u64, 0> > > } } },
    //{ "ADD",    MASK_OPCODE, OPCODE(9),     1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntAdd<u32, S0, S1> > } } },
    //{ "ADDi",   MASK_OPCODE, OPCODE(10),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntAdd<u32, S0, S1> > } } },
    //{ "SUB",    MASK_OPCODE, OPCODE(11),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntSub<u32, S0, S1> > } } },
    //{ "SUBi",   MASK_OPCODE, OPCODE(12),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntSub<u32, S0, S1> > } } },
    //{ "MUL",    MASK_OPCODE, OPCODE(13),    1,{ { OpClassCode::iMUL,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntMul<u32, S0, S1> > } } },
    //{ "MULi",   MASK_OPCODE, OPCODE(14),    1,{ { OpClassCode::iMUL,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntMul<u32, S0, S1> > } } },
    //{ "DIV",    MASK_OPCODE, OPCODE(15),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntDiv<s32, S0, S1> > } } },
    //{ "DIVi",   MASK_OPCODE, OPCODE(16),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntDiv<s32, S0, S1> > } } },
    //{ "DIVU",   MASK_OPCODE, OPCODE(17),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntDiv<u32, S0, S1> > } } },
    //{ "DIVUi",  MASK_OPCODE, OPCODE(18),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntDiv<u32, S0, S1> > } } },
    //{ "MOD",    MASK_OPCODE, OPCODE(19),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64IntMod<s32, S0, S1> > } } },
    //{ "MODi",   MASK_OPCODE, OPCODE(20),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64IntMod<s32, S0, S1> > } } },
    //{ "MODU",   MASK_OPCODE, OPCODE(21),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64IntMod<u32, S0, S1> > } } },
    //{ "MODUi",  MASK_OPCODE, OPCODE(22),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64IntMod<u32, S0, S1> > } } },
    //{ "SLT",    MASK_OPCODE, OPCODE(23),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessSigned<u32> > > } } },
    //{ "SLTi",   MASK_OPCODE, OPCODE(24),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessSigned<u32> > > } } },
    //{ "SLTU",   MASK_OPCODE, OPCODE(25),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessUnsigned<u32> > > } } },
    //{ "SLTUi",  MASK_OPCODE, OPCODE(26),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessUnsigned<u32> > > } } },
    //{ "FTOI",   MASK_OPCODE, OPCODE(27),    1,{ { OpClassCode::ifCONV,{ R0,-1 },{ R1,-1, -1 }, Set<D0, Cast<RegisterType, Cast<s32, AsFP<f32, Cast<s32, S0> > > > >} } },
    //{ "ITOF",   MASK_OPCODE, OPCODE(28),    1,{ { OpClassCode::ifCONV,{ R0,-1 },{ R1,-1, -1 }, Set<D0, Cast<RegisterType, AsInt<s32, Cast<f32, Cast<s32, S0> > > > > } } },
    //{ "FADD",   MASK_OPCODE, OPCODE(29),    1,{ { OpClassCode::fADD,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPAdd<f32, AsFP<f32, Cast<s32, S0> >, AsFP<f32, Cast<s32, S1> >  > > > > } } },
    //{ "FSUB",   MASK_OPCODE, OPCODE(30),    1,{ { OpClassCode::fADD,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPSub<f32, AsFP<f32, Cast<s32, S0> >, AsFP<f32, Cast<s32, S1> >  > > > > } } },
    //{ "FMUL",   MASK_OPCODE, OPCODE(31),    1,{ { OpClassCode::fMUL,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPMul<f32, AsFP<f32, Cast<s32, S0> >, AsFP<f32, Cast<s32, S1> >  > > > > } } },
    //{ "FDIV",   MASK_OPCODE, OPCODE(32),    1,{ { OpClassCode::fDIV,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPDiv<f32, AsFP<f32, Cast<s32, S0> >, AsFP<f32, Cast<s32, S1> >  > > > > } } },
    //{ "FSLT",   MASK_OPCODE, OPCODE(33),    1,{ { OpClassCode::fADD,{ R0,-1 },{ R1,R2,-1 },Set<D0, STRAIGHT64FCompare<f32, AsFP<f32, Cast<s32, S0> >, AsFP<f32, Cast<s32, S1> >  > > } } },
    //{ "SHL",    MASK_OPCODE, OPCODE(34),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, R2, -1 }, Set<D0, LShiftL<u32, S0, S1, 0x3f> > } } },
    //{ "SHLi",   MASK_OPCODE, OPCODE(35),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, I0, -1 }, Set<D0, LShiftL<u32, S0, S1, 0x3f> > } } },
    //{ "SHR",    MASK_OPCODE, OPCODE(36),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, R2, -1 }, Set<D0, LShiftR<u32, S0, S1, 0x3f> > } } },
    //{ "SHRi",   MASK_OPCODE, OPCODE(37),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, I0, -1 }, Set<D0, LShiftR<u32, S0, S1, 0x3f> > } } },
    //{ "SHRA",   MASK_OPCODE, OPCODE(38),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, R2, -1 }, Set<D0, AShiftR<u32, S0, S1, 0x3f> > } } },
    //{ "SHRAi",  MASK_OPCODE, OPCODE(39),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, I0, -1 }, Set<D0, AShiftR<u32, S0, S1, 0x3f> > } } },
    //{ "AND",    MASK_OPCODE, OPCODE(40),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, BitAnd<u32, S0, S1> > } } },
    //{ "ANDi",   MASK_OPCODE, OPCODE(41),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, BitAnd<u32, S0, S1> > } } },
    //{ "OR",     MASK_OPCODE, OPCODE(42),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, BitOr<u32, S0, S1> > } } },
    //{ "ORi",    MASK_OPCODE, OPCODE(43),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, BitOr<u32, S0, S1> > } } },
    //{ "XOR",    MASK_OPCODE, OPCODE(44),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, BitXor<u32, S0, S1> > } } },
    //{ "XORi",   MASK_OPCODE, OPCODE(45),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, BitXor<u32, S0, S1> > } } },
    //{ "LUi",    MASK_OPCODE, OPCODE(46),    1,{ { OpClassCode::iALU,{ R0, -1 },{ I0, -1, -1 }, Set<D0, STRAIGHT64Lui<S0> > } } },
    //{ "SPADDi", MASK_OPCODE, OPCODE(47),    1,{ { OpClassCode::iMOV,{ R0, -1 },{ I0, -1, -1 }, Set<D0, STRAIGHT64Copy<S0> > } } }, // iMOV扱いだがフロントエンドで即値を書き換える
    //{ "RPINC",  MASK_OPCODE, OPCODE(48),    1,{ { OpClassCode::iNOP,{ -1, -1 },{ I0, -1, -1 }, NoOperation } } },                  // フロントエンドで処理するのでNOP扱い
    //{ "RMOV",   MASK_OPCODE, OPCODE(49),    1,{ { OpClassCode::iMOV,{ R0, -1 },{ R1, -1, -1 }, Set<D0, STRAIGHT64Copy<S0> > } } },
    //{ "LD",     MASK_OPCODE, OPCODE(50),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<u32, STRAIGHT64Addr<S0, S1> > > } } },
    //{ "LDH",    MASK_OPCODE, OPCODE(51),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<s32, STRAIGHT64Addr<S0, S1> > > } } },
    //{ "LDHU",   MASK_OPCODE, OPCODE(52),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<u32, STRAIGHT64Addr<S0, S1> > > } } },
    //{ "LDB",    MASK_OPCODE, OPCODE(53),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<s8,  STRAIGHT64Addr<S0, S1> > > } } },
    //{ "LDBU",   MASK_OPCODE, OPCODE(54),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<u8,  STRAIGHT64Addr<S0, S1> > > } } },
    //{ "ST",     MASK_OPCODE, OPCODE(55),    1,{ { OpClassCode::iST,{ R0, -1 },{ R1, I0, R2 }, Set<D0, STRAIGHT64Store<u32, S0, STRAIGHT64Addr<S1, S2> > > } } },
    //{ "STH",    MASK_OPCODE, OPCODE(56),    1,{ { OpClassCode::iST,{ R0, -1 },{ R1, I0, R2 }, Set<D0, STRAIGHT64Store<s32, S0, STRAIGHT64Addr<S1, S2> > > } } },
    //{ "STB",    MASK_OPCODE, OPCODE(57),    1,{ { OpClassCode::iST,{ R0, -1 },{ R1, I0, R2 }, Set<D0, STRAIGHT64Store<s8,  S0, STRAIGHT64Addr<S1, S2> > > } } }
    
};

//
// STRAIGHT64Converter
//


STRAIGHT64Converter::STRAIGHT64Converter()
{
    AddToOpMap(m_OpDefsBase, sizeof(m_OpDefsBase)/sizeof(OpDef));
}

STRAIGHT64Converter::~STRAIGHT64Converter()
{
}

// srcTemplate に対応するオペランドの種類と，レジスタならば番号を，即値ならばindexを返す
std::pair<STRAIGHT64Converter::OperandType, int> STRAIGHT64Converter::GetActualSrcOperand(int srcTemplate, const DecodedInsn& decoded) const
{
    typedef std::pair<OperandType, int> RetType;
    if (srcTemplate == -1) {
        return RetType(OpInfoType::NONE, 0);
    }
    else if (ImmTemplateBegin <= srcTemplate && srcTemplate <= ImmTemplateEnd) {
        return RetType(OpInfoType::IMM, srcTemplate - ImmTemplateBegin);
    }
    else  {
        return RetType(OpInfoType::REG, GetActualRegNumber(srcTemplate, decoded) );
    }
}

// regTemplate から実際のレジスタ番号を取得する
int STRAIGHT64Converter::GetActualRegNumber(int regTemplate, const DecodedInsn& decoded) const
{
    if (regTemplate == -1) {
        return -1;
    }
    else if (RegTemplateBegin <= regTemplate && regTemplate <= RegTemplateEnd) {
        return decoded.Reg[regTemplate - RegTemplateBegin];
    }
    else if (0 <= regTemplate && regTemplate < STRAIGHT64Info::RegisterCount) {
        return regTemplate;
    }
    else {
        ASSERT(0, "STRAIGHT64Converter Logic Error : invalid regTemplate");
        return -1;
    }
}

// レジスタ番号regNumがゼロレジスタかどうかを判定する
bool STRAIGHT64Converter::IsZeroReg(int regNum) const
{
    return regNum == STRAIGHT64Info::ZeroRegIndex;
}


void STRAIGHT64Converter::STRAIGHTUnknownOperation(OpEmulationState* opState)
{
    u32 codeWord = static_cast<u32>( SrcOperand<0>()(opState) );

    DecoderType decoder;
    DecodedInsn decoded;
    decoder.Decode( codeWord, &decoded);

    stringstream ss;
    u32 opcode = (codeWord & 0xfc000000) >> 26;
    ss << "unknown instruction : " << hex << setw(8) << codeWord << endl;
    ss << "\topcode : " << hex << opcode << endl;
    ss << "\timm[1] : " << hex << decoded.Imm[1] << endl;
    ss << "(PC = " << hex << opState->GetPC() << ")" << endl;

    THROW_RUNTIME_ERROR(ss.str().c_str());
}

const STRAIGHT64Converter::OpDef* STRAIGHT64Converter::GetOpDefUnknown() const
{
    return &m_OpDefUnknown;
}
