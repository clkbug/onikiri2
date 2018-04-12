//
// Copyright (c) 2005-2008 Kenichi Watanabe.
// Copyright (c) 2005-2008 Yasuhiro Watari.
// Copyright (c) 2005-2008 Hironori Ichibayashi.
// Copyright (c) 2008-2009 Kazuo Horio.
// Copyright (c) 2009-2015 Naruki Kurata.
// Copyright (c) 2005-2015 Ryota Shioya.
// Copyright (c) 2005-2015 Masahiro Goshima.
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
#include "Emu/STRAIGHT64Linux/STRAIGHTSystem.h"
#include "Emu/STRAIGHT64Linux/STRAIGHT64LinuxEmulator.h"
#include "Sim/Foundation/Hook/Hook.h"
#include "Sim/Pipeline/Fetcher/Fetcher.h"
#include "Sim/Pipeline/Renamer/Renamer.h"
#include "Sim/Dumper/Dumper.h"
#include "Sim/InorderList/InorderList.h"
#include "Sim/System/ForwardEmulator.h"

using namespace std;

using namespace Onikiri;
using namespace STRAIGHT;

STRAIGHTSystem::STRAIGHTSystem() :
    m_emuRP(0),
    m_emuSP(INITIAL_SP),
    m_core(nullptr)
{
}

STRAIGHTSystem::~STRAIGHTSystem()
{    
}



void STRAIGHTSystem::Initialize(InitPhase phase)
{
    if (phase == INIT_PRE_CONNECTION)
    {
        LoadParam();
    }
    else if (phase == INIT_POST_CONNECTION)
    {
        CheckNodeInitialized("core", m_core);

        m_spTable.Resize(*m_core->GetOpArray());
        m_rpTable.Resize(*m_core->GetOpArray());

        //STRAIGHTEmulator::s_getOpHook.Register(this, &STRAIGHTSystem::AfterEmulatorGetOp, 0, HookType::HOOK_AFTER);
        //Fetcher::s_fetchHook.Register(this, &STRAIGHTSystem::OnFetch, 0, HookType::HOOK_AFTER);
        //Fetcher::s_getOpHook.Register(this, &STRAIGHTSystem::AfterFetcherGetOp, 0, HookType::HOOK_AFTER);
        //ForwardEmulator::s_getOpHook.Register(this, &STRAIGHTSystem::AfterForwardEmulatorGetOp, 0, HookType::HOOK_AFTER);
        //InorderList::s_opFlushHook.Register(this, &STRAIGHTSystem::BeforeFlush, 0, HookType::HOOK_BEFORE);
    }
    
}

void STRAIGHTSystem::Finalize()
{
    ReleaseParam();    
}

void STRAIGHTSystem::ChangeSimulationMode(SimulationMode mode)
{
    if (mode == SM_SIMULATION)
    {
        m_rp = m_emuRP; // m_emuRPはEmulationの時から使われていて正しい値を保持しているのでSimulation用にコピーする
        m_sp = m_emuSP; // m_emuSPも同様
    }
}

void STRAIGHTSystem::Rename(std::pair<OpInfo**, int>* ops, u64* rp)
{
    //auto opInfo = dynamic_cast<STRAIGHTOpInfo*>(*(*ops).first);
    //if(!opInfo)
    //{
    //    THROW_RUNTIME_ERROR("STRAIGHTSystemがSTRAIGHTOpInfoでないものを掴まされた．");
    //}

    //opInfo->SetDstOperand(0, static_cast<int>(*rp));
    //for (int i = 0; i < opInfo->GetSrcNum(); i++)
    //{
    //    auto d = opInfo->GetSrcOperand(i);
    //    if (d == STRAIGHTISAInfo::ZEROREG_INDEX) { continue; }
    //    auto src = STRAIGHTISAInfo::CalcRP(*rp, -d);
    //    opInfo->SetSrcOperand(i, static_cast<int>(src));
    //}
}

//void STRAIGHTSystem::AfterEmulatorGetOp(STRAIGHTEmulator::GetOpHookParam* param)
//{
//    Rename(param, &m_emuRP);
//
//    auto opInfo = dynamic_cast<STRAIGHTOpInfo*>(*(param->first));
//    if (!opInfo)
//    {
//        THROW_RUNTIME_ERROR("STRAIGHTSystemがSTRAIGHTOpInfoでないものを掴まされた．");
//    }
//
//    // Process RPINC
//    if (opInfo->GetOpCode() == OPCODE_RPINC)
//    {
//        m_emuRP = STRAIGHTISAInfo::CalcRP(m_emuRP, opInfo->GetImmValue() + 1);
//    }
//    else
//    {
//        m_emuRP = STRAIGHTISAInfo::CalcRP(m_emuRP, 1);
//    }
//
//    // Process SPADDi
//    if (opInfo->GetOpCode() == OPCODE_SPADD_IMM)
//    {
//        opInfo->SetSP(m_emuSP);
//        m_emuSP += opInfo->GetImmValue();
//    }
//}

//void STRAIGHTSystem::AfterFetcherGetOp(Fetcher::GetOpHookParam* param)
//{
//    Rename(param, &m_rp);
//}
//void STRAIGHTSystem::AfterForwardEmulatorGetOp(ForwardEmulator::GetOpHookParam* param)
//{
//    Rename(param, &m_emuRP);
//
//    auto opInfo = dynamic_cast<STRAIGHTOpInfo*>(*(*param).first);
//    if(!opInfo)
//    {
//        THROW_RUNTIME_ERROR("STRAIGHTSystemがSTRAIGHTOpInfoでないものを掴まされた．");
//    }
//
//    // Process RPINC
//    if (opInfo->GetOpCode() == OPCODE_RPINC)
//    {
//        m_emuRP = STRAIGHTISAInfo::CalcRP(m_emuRP, opInfo->GetImmValue() + 1);
//    }
//    else
//    {
//        m_emuRP = STRAIGHTISAInfo::CalcRP(m_emuRP, 1);
//    }
//    
//    // Process SPADDi
//    if (opInfo->GetOpCode() == OPCODE_SPADD_IMM)
//    {
//        opInfo->SetSP(m_emuSP);
//        m_emuSP += opInfo->GetImmValue();
//    }
//    
//}
//
//void STRAIGHTSystem::OnFetch(Fetcher::FetchHookParam* param)
//{
//    auto op = param->op;
//
//    auto opInfo = dynamic_cast<STRAIGHTOpInfo*>(op->GetOpInfo());
//    if (!opInfo)
//    {
//        THROW_RUNTIME_ERROR("STRAIGHTSystemがSTRAIGHTOpInfoでないものを掴まされた．");
//    }
//
//    ASSERT( opInfo->GetDstOperand(0) == static_cast<int>(m_rp) );
//    m_rpTable[op] = m_rp;
//    m_spTable[op] = m_sp;
//
//    // Process RPINC
//    if (opInfo->GetOpCode() == OPCODE_RPINC)
//    {
//        m_rp = STRAIGHTISAInfo::CalcRP(m_rp, opInfo->GetImmValue() + 1);
//    }
//    else
//    {
//        m_rp = STRAIGHTISAInfo::CalcRP(m_rp, 1);
//    }
//
//    // Process SPADDi
//    if (opInfo->GetOpCode() == OPCODE_SPADD_IMM)
//    {
//        opInfo->SetSP(m_sp);
//        m_sp += opInfo->GetImmValue();
//    }
//}

void STRAIGHTSystem::BeforeFlush(OpIterator op)
{
    m_rp = m_rpTable[op];
    m_sp = m_spTable[op];
}
