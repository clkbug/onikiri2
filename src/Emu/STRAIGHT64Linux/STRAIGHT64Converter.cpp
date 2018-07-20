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

using namespace std;
using namespace boost;
using namespace Onikiri;
using namespace Onikiri::EmulatorUtility;
using namespace Onikiri::STRAIGHT64Linux;
using namespace Onikiri::EmulatorUtility::Operation;
using namespace Onikiri::STRAIGHT64Linux::Operation;

//
// 命令の定義
//

namespace {

    // 各命令形式に対するオペコードを得るためのマスク (0のビットが引数)
    const u32 MASK_EXACT = 0xffffffff;  // 全bitが一致
    const u32 MASK_OPCODE = 0xfc000000; // 最上位6ビットがOPCODE
}

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

    const int I0 = ImmTemplateBegin+0;
}

#define STRAIGHT64_DSTOP(n) STRAIGHT64DstOperand<n>
#define STRAIGHT64_SRCOP(n) STRAIGHT64SrcOperand<n>
#define OPCODE(n) ((unsigned)(n)<<26)

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
    { "NOP",    MASK_OPCODE,  OPCODE(0),     1,   { { OpClassCode::iNOP,  { -1, -1 }, { -1, -1, -1 }, NoOperation } } },
    { "SYSCALL",MASK_OPCODE, OPCODE(1),     2,   {
        { OpClassCode::syscall,        { R0, -1 }, { I0,  1,  2 }, STRAIGHT64SyscallSetArg },
        { OpClassCode::syscall_branch, { R0, -1 }, { R0,  3,  4 }, STRAIGHT64SyscallCore },
    } },
    { "J",      MASK_OPCODE, OPCODE(3),     1,  { { OpClassCode::iJUMP,  { R0, -1 }, { I0, -1, -1 }, Sequence2<BranchRelUncond<S0>, Set<D0, IntConst<u64, 0> > > } } },
    { "JR",     MASK_OPCODE, OPCODE(4),     1,  { { OpClassCode::iJUMP,  { R0, -1 }, { R1, -1, -1 }, Sequence2<BranchAbsUncond<S0>, Set<D0, IntConst<u64, 0> > > } } },
    { "JAL",    MASK_OPCODE, OPCODE(5),     1,  { { OpClassCode::CALL,   { R0, -1 }, { I0, -1, -1 }, CallRelUncond<D0, S0> } } },
    { "JRAL",   MASK_OPCODE, OPCODE(6),     1,  { { OpClassCode::CALL_JUMP,  { R0, -1 }, { R1, -1, -1 }, CallAbsUncond<D0, S0> } } },
    { "BEZ",    MASK_OPCODE, OPCODE(7),     1,  { { OpClassCode::iBC,    { R0, -1 }, { R1, I0, -1 }, Sequence2<BranchRelCond< S1, Compare< S0, IntConst<u64, 0>, IntCondEqual<u64> > >, Set<D0, IntConst<u64, 0> > > } } },
    { "BNZ",    MASK_OPCODE, OPCODE(8),     1,  { { OpClassCode::iBC,    { R0, -1 }, { R1, I0, -1 }, Sequence2<BranchRelCond< S1, Compare< S0, IntConst<u64, 0>, IntCondNotEqual<u64> > >, Set<D0, IntConst<u64, 0> > > } } },
    { "ADD",    MASK_OPCODE, OPCODE(9),     1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntAdd<u32, S0, S1> > } } },
    { "ADDi",   MASK_OPCODE, OPCODE(10),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntAdd<u32, S0, S1> > } } },
    { "SUB",    MASK_OPCODE, OPCODE(11),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntSub<u32, S0, S1> > } } },
    { "SUBi",   MASK_OPCODE, OPCODE(12),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntSub<u32, S0, S1> > } } },
    { "MUL",    MASK_OPCODE, OPCODE(13),    1,{ { OpClassCode::iMUL,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntMul<u32, S0, S1> > } } },
    { "MULi",   MASK_OPCODE, OPCODE(14),    1,{ { OpClassCode::iMUL,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntMul<u32, S0, S1> > } } },
    { "DIV",    MASK_OPCODE, OPCODE(15),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntDiv<s32, S0, S1> > } } },
    { "DIVi",   MASK_OPCODE, OPCODE(16),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntDiv<s32, S0, S1> > } } },
    { "DIVU",   MASK_OPCODE, OPCODE(17),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, IntDiv<u32, S0, S1> > } } },
    { "DIVUi",  MASK_OPCODE, OPCODE(18),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, IntDiv<u32, S0, S1> > } } },
    { "MOD",    MASK_OPCODE, OPCODE(19),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64IntMod<s32, S0, S1> > } } },
    { "MODi",   MASK_OPCODE, OPCODE(20),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64IntMod<s32, S0, S1> > } } },
    { "MODU",   MASK_OPCODE, OPCODE(21),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64IntMod<u32, S0, S1> > } } },
    { "MODUi",  MASK_OPCODE, OPCODE(22),    1,{ { OpClassCode::iDIV,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64IntMod<u32, S0, S1> > } } },
    { "SLT",    MASK_OPCODE, OPCODE(23),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessSigned<u32> > > } } },
    { "SLTi",   MASK_OPCODE, OPCODE(24),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessSigned<u32> > > } } },
    { "SLTU",   MASK_OPCODE, OPCODE(25),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessUnsigned<u32> > > } } },
    { "SLTUi",  MASK_OPCODE, OPCODE(26),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, STRAIGHT64Compare<S0, S1, IntCondLessUnsigned<u32> > > } } },
    // Float: not implemented yet
{ "FTOI",   MASK_OPCODE, OPCODE(27),    1,{ { OpClassCode::ifCONV,{ R0,-1 },{ R1,-1, -1 },Set<D0, Cast <RegisterType, Cast<s32, AsFP<f32, Cast<u32, S0> > > > > } } },
{ "FTOI",   MASK_OPCODE, OPCODE(28),    1,{ { OpClassCode::ifCONV,{ R0,-1 },{ R1,-1, -1 },Set<D0, Cast <RegisterType, Cast<f32, AsInt<s32, Cast<u32, S0> > > > > } } },
{ "FADD",   MASK_OPCODE, OPCODE(29),    1,{ { OpClassCode::fADD,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPAdd<f32, AsFP<f32, Cast<u32, S0> >, AsFP<f32, Cast<u32, S1> >  > > > > } } },
{ "FSUB",   MASK_OPCODE, OPCODE(30),    1,{ { OpClassCode::fADD,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPSub<f32, AsFP<f32, Cast<u32, S0> >, AsFP<f32, Cast<u32, S1> >  > > > > } } },
{ "FMUL",   MASK_OPCODE, OPCODE(31),    1,{ { OpClassCode::fMUL,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPMul<f32, AsFP<f32, Cast<u32, S0> >, AsFP<f32, Cast<u32, S1> >  > > > > } } },
{ "FDIV",   MASK_OPCODE, OPCODE(32),    1,{ { OpClassCode::fDIV,{ R0,-1 },{ R1,R2,-1 },Set<D0, Cast <RegisterType, AsInt<u32, FPDiv<f32, AsFP<f32, Cast<u32, S0> >, AsFP<f32, Cast<u32, S1> >  > > > > } } },
    { "SHL",    MASK_OPCODE, OPCODE(34),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, R2, -1 }, Set<D0, LShiftL<u32, S0, S1, 0x3f> > } } },
    { "SHLi",   MASK_OPCODE, OPCODE(35),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, I0, -1 }, Set<D0, LShiftL<u32, S0, S1, 0x3f> > } } },
    { "SHR",    MASK_OPCODE, OPCODE(36),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, R2, -1 }, Set<D0, LShiftR<u32, S0, S1, 0x3f> > } } },
    { "SHRi",   MASK_OPCODE, OPCODE(37),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, I0, -1 }, Set<D0, LShiftR<u32, S0, S1, 0x3f> > } } },
    { "SHRA",   MASK_OPCODE, OPCODE(38),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, R2, -1 }, Set<D0, AShiftR<u32, S0, S1, 0x3f> > } } },
    { "SHRAi",  MASK_OPCODE, OPCODE(39),    1,{ { OpClassCode::iSFT,{ R0, -1 },{ R1, I0, -1 }, Set<D0, AShiftR<u32, S0, S1, 0x3f> > } } },
    { "AND",    MASK_OPCODE, OPCODE(40),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, BitAnd<u32, S0, S1> > } } },
    { "ANDi",   MASK_OPCODE, OPCODE(41),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, BitAnd<u32, S0, S1> > } } },
    { "OR",     MASK_OPCODE, OPCODE(42),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, BitOr<u32, S0, S1> > } } },
    { "ORi",    MASK_OPCODE, OPCODE(43),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, BitOr<u32, S0, S1> > } } },
    { "XOR",    MASK_OPCODE, OPCODE(44),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, R2, -1 }, Set<D0, BitXor<u32, S0, S1> > } } },
    { "XORi",   MASK_OPCODE, OPCODE(45),    1,{ { OpClassCode::iALU,{ R0, -1 },{ R1, I0, -1 }, Set<D0, BitXor<u32, S0, S1> > } } },
    { "LUi",    MASK_OPCODE, OPCODE(46),    1,{ { OpClassCode::iALU,{ R0, -1 },{ I0, -1, -1 }, Set<D0, STRAIGHT64Lui<S0> > } } },
    { "SPADDi", MASK_OPCODE, OPCODE(47),    1,{ { OpClassCode::iMOV,{ R0, -1 },{ I0, -1, -1 }, Set<D0, STRAIGHT64Copy<S0> > } } }, // iMOV扱いだがフロントエンドで即値を書き換える
    { "RPINC",  MASK_OPCODE, OPCODE(48),    1,{ { OpClassCode::iNOP,{ -1, -1 },{ I0, -1, -1 }, NoOperation } } },                  // フロントエンドで処理するのでNOP扱い
    { "RMOV",   MASK_OPCODE, OPCODE(49),    1,{ { OpClassCode::iMOV,{ R0, -1 },{ R1, -1, -1 }, Set<D0, STRAIGHT64Copy<S0> > } } },
    { "LD",     MASK_OPCODE, OPCODE(50),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<u32, STRAIGHT64Addr<S0, S1> > > } } },
    { "LDH",    MASK_OPCODE, OPCODE(51),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<s32, STRAIGHT64Addr<S0, S1> > > } } },
    { "LDHU",   MASK_OPCODE, OPCODE(52),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<u32, STRAIGHT64Addr<S0, S1> > > } } },
    { "LDB",    MASK_OPCODE, OPCODE(53),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<s8,  STRAIGHT64Addr<S0, S1> > > } } },
    { "LDBU",   MASK_OPCODE, OPCODE(54),    1,{ { OpClassCode::iLD,{ R0, -1 },{ R1, I0, -1 }, Set<D0, Load<u8,  STRAIGHT64Addr<S0, S1> > > } } },
    { "ST",     MASK_OPCODE, OPCODE(55),    1,{ { OpClassCode::iST,{ R0, -1 },{ R1, I0, R2 }, Set<D0, STRAIGHT64Store<u32, S0, STRAIGHT64Addr<S1, S2> > > } } },
    { "STH",    MASK_OPCODE, OPCODE(56),    1,{ { OpClassCode::iST,{ R0, -1 },{ R1, I0, R2 }, Set<D0, STRAIGHT64Store<s32, S0, STRAIGHT64Addr<S1, S2> > > } } },
    { "STB",    MASK_OPCODE, OPCODE(57),    1,{ { OpClassCode::iST,{ R0, -1 },{ R1, I0, R2 }, Set<D0, STRAIGHT64Store<s8,  S0, STRAIGHT64Addr<S1, S2> > > } } }
    
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
