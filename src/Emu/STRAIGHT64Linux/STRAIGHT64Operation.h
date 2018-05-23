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


#ifndef EMU_STRAIGHT64LINUX_STRAIGHT64_OPERATION_H
#define EMU_STRAIGHT64LINUX_STRAIGHT64_OPERATION_H

#include "SysDeps/fenv.h"
#include "Utility/RuntimeError.h"
#include "Emu/Utility/GenericOperation.h"
#include "Emu/Utility/System/Syscall/SyscallConvIF.h"
#include "Emu/Utility/System/ProcessState.h"


namespace Onikiri {
namespace STRAIGHT64Linux {
namespace Operation {

using namespace EmulatorUtility;
using namespace EmulatorUtility::Operation;

typedef u32 STRAIGHT64RegisterType;

static const STRAIGHT64RegisterType REG_VALUE_TRUE = 1;
static const STRAIGHT64RegisterType REG_VALUE_FALSE = 0;

// Operands
template <int OperandIndex>
class STRAIGHT64DstOperand
{
public:
    typedef STRAIGHT64RegisterType type;
    static void SetOperand(OpEmulationState* opState, STRAIGHT64RegisterType value)
    {
        opState->SetDst(OperandIndex, value);
    }
};

template <int OperandIndex>
struct STRAIGHT64SrcOperand : public std::unary_function<OpEmulationState, STRAIGHT64RegisterType>
{
    typedef STRAIGHT64RegisterType type;
    STRAIGHT64RegisterType operator()(OpEmulationState* opState)
    {
        return (STRAIGHT64RegisterType)opState->GetSrc(OperandIndex);
    }
};

template <typename Type, Type value>
struct STRAIGHT64IntConst : public std::unary_function<OpEmulationState*, Type>
{
    Type operator()(OpEmulationState* opState)
    {
        return value;
    }
};


// interface
inline void STRAIGHT64DoBranch(OpEmulationState* opState, u32 target)
{
    opState->SetTaken(true);
    opState->SetTakenPC(target);
}

inline u32 STRAIGHT64NextPC(OpEmulationState* opState)
{
    return (u32)opState->GetPC() + 4;
}

inline u32 STRAIGHT64CurPC(OpEmulationState* opState)
{
    return (u32)opState->GetPC();
}


// Set PC
template<typename TSrc1>
struct STRAIGHT64Auipc : public std::unary_function<OpEmulationState, RegisterType>
{
    STRAIGHT64RegisterType operator()(OpEmulationState* opState) const
    {
        return (TSrc1()(opState) << 12) + STRAIGHT64CurPC(opState);
    }
};

// Load upper immediate
template<typename TSrc1>
struct STRAIGHT64Lui : public std::unary_function<OpEmulationState, RegisterType>
{
    STRAIGHT64RegisterType operator()(OpEmulationState* opState) const
    {
        return TSrc1()(opState) << 12;
    }
};


// compare
template <typename TSrc1, typename TSrc2, typename Comp>
struct STRAIGHT64Compare : public std::unary_function<EmulatorUtility::OpEmulationState*, STRAIGHT64RegisterType>
{
    STRAIGHT64RegisterType operator()(EmulatorUtility::OpEmulationState* opState)
    {
        if (Comp()(TSrc1()(opState), TSrc2()(opState))) {
            return (STRAIGHT64RegisterType)REG_VALUE_TRUE;
        }
        else {
            return (STRAIGHT64RegisterType)REG_VALUE_FALSE;
        }
    }
};

// Branch
template <typename TSrcTarget, typename TSrcDisp>
inline void STRAIGHT64BranchAbsUncond(OpEmulationState* opState)
{
    u32 target = TSrcTarget()(opState) + cast_to_signed(TSrcDisp()(opState));
    STRAIGHT64DoBranch(opState, target);
}

template <typename TSrcDisp>
inline void STRAIGHT64BranchRelUncond(OpEmulationState* opState)
{
    u32 target = STRAIGHT64CurPC(opState) + cast_to_signed(TSrcDisp()(opState));
    STRAIGHT64DoBranch(opState, target);
}

template <typename TSrcDisp, typename TCond>
inline void STRAIGHT64BranchRelCond(OpEmulationState* opState)
{
    if (TCond()(opState)) {
        STRAIGHT64BranchRelUncond<TSrcDisp>(opState);
    }
    else {
        opState->SetTakenPC(STRAIGHT64NextPC(opState));
    }
}


template <typename TDest, typename TSrcDisp>
inline void STRAIGHT64CallRelUncond(OpEmulationState* opState)
{
    STRAIGHT64RegisterType ret = static_cast<STRAIGHT64RegisterType>(STRAIGHT64NextPC(opState));
    STRAIGHT64BranchRelUncond<TSrcDisp>(opState);
    TDest::SetOperand(opState, ret);
}

template <typename TDest, typename TSrcTarget, typename TSrcDisp>
inline void STRAIGHT64CallAbsUncond(OpEmulationState* opState)
{
    STRAIGHT64RegisterType ret = static_cast<STRAIGHT64RegisterType>(STRAIGHT64NextPC(opState));
    STRAIGHT64BranchAbsUncond<TSrcTarget, TSrcDisp>(opState);
    TDest::SetOperand(opState, ret);
}

//
// Load/Store
//
template<typename TSrc1, typename TSrc2>
struct STRAIGHT64Addr : public std::unary_function<EmulatorUtility::OpEmulationState, STRAIGHT64RegisterType>
{
    STRAIGHT64RegisterType operator()(EmulatorUtility::OpEmulationState* opState) const
    {
        return TSrc1()(opState) + EmulatorUtility::cast_to_signed(TSrc2()(opState));
    }
};

//
// Div/Rem
//
template <typename TSrc1, typename TSrc2>
struct STRAIGHT64IntDiv : public std::unary_function<OpEmulationState*, u32>
{
    u32 operator()(OpEmulationState* opState)
    {
        s32 src1 = static_cast<s32>(TSrc1()(opState));
        s32 src2 = static_cast<s32>(TSrc2()(opState));
        if (src2 == 0){
            return static_cast<u32>(-1);
        }
        if (src1 < -0x7fffffff && src2 == -1) { // overflow
            return src1;
        }
        return static_cast<u32>(src1 / src2);
    }
};

template <typename TSrc1, typename TSrc2>
struct STRAIGHT64IntRem : public std::unary_function<OpEmulationState*, u32>
{
    u32 operator()(OpEmulationState* opState)
    {
        s32 src1 = static_cast<s32>(TSrc1()(opState));
        s32 src2 = static_cast<s32>(TSrc2()(opState));
        if (src2 == 0){
            return src1;
        }
        if (src1 < -0x7fffffff && src2 == -1) { // overflow
            return 0;
        }
        return src1 % src2;
    }
};

template <typename TSrc1, typename TSrc2>
struct STRAIGHT64IntDivu : public std::unary_function<OpEmulationState*, u32>
{
    u32 operator()(OpEmulationState* opState)
    {
        u32 src1 = static_cast<u32>(TSrc1()(opState));
        u32 src2 = static_cast<u32>(TSrc2()(opState));
        if (src2 == 0){
            return static_cast<u32>(-1);
        }
        return src1 / src2;
    }
};

template <typename TSrc1, typename TSrc2>
struct STRAIGHT64IntRemu : public std::unary_function<OpEmulationState*, u32>
{
    u32 operator()(OpEmulationState* opState)
    {
        u32 src1 = static_cast<u32>(TSrc1()(opState));
        u32 src2 = static_cast<u32>(TSrc2()(opState));
        if (src2 == 0){
            return src1;
        }
        return src1 % src2;
    }
};


void STRAIGHT64SyscallSetArg(EmulatorUtility::OpEmulationState* opState)
{
    EmulatorUtility::SyscallConvIF* syscallConv = opState->GetProcessState()->GetSyscallConv();
    syscallConv->SetArg(0, SrcOperand<0>()(opState));
    syscallConv->SetArg(1, SrcOperand<1>()(opState));
    syscallConv->SetArg(2, SrcOperand<2>()(opState));
    DstOperand<0>::SetOperand(opState, SrcOperand<0>()(opState));
}

// invoke syscall, get result&error and branch if any
void STRAIGHT64SyscallCore(EmulatorUtility::OpEmulationState* opState)
{
    EmulatorUtility::SyscallConvIF* syscallConv = opState->GetProcessState()->GetSyscallConv();
    syscallConv->SetArg(3, SrcOperand<1>()(opState));
    syscallConv->SetArg(4, SrcOperand<2>()(opState));
    //syscallConv->SetArg(5, SrcOperand<2>()(opState));
    syscallConv->Execute(opState);

    u32 error = (u32)syscallConv->GetResult(EmulatorUtility::SyscallConvIF::ErrorFlagIndex);
    u32 val = (u32)syscallConv->GetResult(EmulatorUtility::SyscallConvIF::RetValueIndex);
    DstOperand<0>::SetOperand(opState, error ? (u32)-1 : val);
    //DstOperand<1>::SetOperand(opState, syscallConv->GetResult(EmulatorUtility::SyscallConvIF::ErrorFlagIndex) );
}

} // namespace Operation {
} // namespace STRAIGHT64Linux {
} // namespace Onikiri

#endif // #ifndef EMU_STRAIGHT64LINUX_STRAIGHT64_OPERATION_H

