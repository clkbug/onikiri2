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
    const int RegTemplateEnd = RegTemplateBegin+4-1;
    const int ImmTemplateBegin = -30;    // 即値に変換
    const int ImmTemplateEnd = ImmTemplateBegin+2-1;

    const int R0 = RegTemplateBegin+0;
    const int R1 = RegTemplateBegin+1;
    const int R2 = RegTemplateBegin+2;

    const int I0 = ImmTemplateBegin+0;
}

#define STRAIGHT64_DSTOP(n) STRAIGHT64DstOperand<n>
#define STRAIGHT64_SRCOP(n) STRAIGHT64SrcOperand<n>
#define OPCODE(n) ((n)<<26)

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
    //{Name,    Mask,        Opcode,        nOp,{ OpClassCode,          Dst[],      Src[],              OpInfoType::EmulationFunc}[]}
    {"ADD",     MASK_OPCODE, OPCODE( 9),    1,  { {OpClassCode::iALU,   {R0, -1},   {R1, R2, -1},       Set<D0, IntAdd<u32, S0, S1> >} } },
    {"ADDi",    MASK_OPCODE, OPCODE(10),    1,  { {OpClassCode::iALU,   {R0, -1},   {R1, I0, -1},       Set<D0, IntAdd<u32, S0, S1> > } } },
    {"SUB",     MASK_OPCODE, OPCODE(11),    1,  { {OpClassCode::iALU,   {R0, -1},   {R1, R2, -1},       Set<D0, IntSub<u32, S0, S1> >} } },
    {"SUBi",    MASK_OPCODE, OPCODE(12),    1,  { {OpClassCode::iALU,   {R0, -1},   {R1, I0, -1},       Set<D0, IntSub<u32, S0, S1> > } } },
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
