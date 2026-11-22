#!/usr/bin/env python3
"""The owner programs PIPE_WM_LINETIME through the native writer before its latch proof.

Contract and basis: the native plane-only partial modeset recalculates the pipe line time without programming
it; only the native pipe enable, mode, stream and underscan paths copy it to the register. An ADL-P eDP
2560x1440 panel (htotal 2720, 245.5MHz) booted directly into a 3072x1728 source calculated 62 (0x3E,
353516853Hz) while PIPE_WM_LINETIME kept the firmware's 89 (0x59), and the latch proof read back 89 against 62
on its first transaction. Entered from the native mode the same source calculated 89 before the scaler took
effect and the native underscan path programmed 89; between two downscaled sources it calculated and
programmed 71 (2880x1620, 310710937Hz) or 62.

Tested path: production adlnProgramPipeLineTime, from the per-pipe record wrapAlderLakeNPipeWMSetup keeps and
the native rate routine's null-CRTC form, through the native writer slot, to the register.

Counterexamples: an equal register is not rewritten; without a record the unscaled rule decides; an
incomplete plan, a missing or short scaler, a short path, an unqualified access object or writer slot, and an
invalid pipe write nothing and never call the rate routine or the writer; a write that does not stick and an
unpowered pipe do not report completion; other pipes' registers are never touched.

Limits: the caller's admission (pending transaction, association, bound writer mapping, non-DSI target) and
the proof that follows are not executed here; the rate routine and the writer are models.
"""
from pathlib import Path
from production_source import read_source
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=read_source(root/'WhateverXe/kern_xe.cpp')
body=re.search(r'__attribute__\(\(noinline\)\) static Display13WM::Program adlnProgramPipeLineTime\(.*?\n\}',s,re.S)[0]
code=r'''
#include "kern_display13_fb_memory.hpp"
#include "kern_display13_wm.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>
using mach_vm_address_t=uint64_t;
static std::map<const void*,unsigned> sizes;
struct OSObject {OSObject *getMetaClass(){return this;}unsigned getClassSize(){return sizes.count(this)?sizes[this]:0;}};
using IOService=OSObject;
struct IOMemoryMap {
 std::vector<uint32_t> words=std::vector<uint32_t>(0x1000000/4);unsigned length=0x1000000;
 uintptr_t getVirtualAddress(){return reinterpret_cast<uintptr_t>(words.data());}
 unsigned getLength(){return length;}
 uint32_t &reg(unsigned r){return words[r/4];}
};
struct Pipe {Display13WM::Plan wm[2];Display13WM::PipeTiming pipeTiming;};
struct {IOService *controller{};IOMemoryMap *map{};void *access{};Pipe pipe[3];}display13FBPower;
static std::vector<uint64_t> published;static unsigned publications;
void adlnPublishMemory(const char *name,unsigned,void *data,unsigned bytes,bool=false){
 assert(!strcmp(name,"N97FBPipeLineTimeProgram"));++publications;
 published.assign(static_cast<uint64_t*>(data),static_cast<uint64_t*>(data)+bytes/8);
}
BODY
alignas(16) static uint8_t fb[0x4B90],path[0x3730],scaler[0x58],accessObject[0x68],controller[0x1C20];
static uintptr_t table[0x138/8];
struct Written {uint64_t reg;uint32_t value;};
static std::vector<Written> writes;static bool dropWrites;static IOMemoryMap *mapped;
static void writer(void *access,uint64_t reg,uint32_t value){
 assert(access==accessObject && reg<0x1000000 && !(reg&3));writes.push_back({reg,value});
 if(!dropWrites)mapped->reg(static_cast<unsigned>(reg))=value;
}
static void otherWriter(void*,uint64_t,uint32_t){assert(false);}
static uint64_t nullHz;static unsigned rateCalls;
static uint64_t rate(void *ctl,void *f,void *p,bool pipe,void *crtc){
 assert(ctl==controller && f==fb && p==path && pipe && !crtc);++rateCalls;return nullHz;
}
template<class T>void put(void *p,unsigned off,T v){memcpy(static_cast<uint8_t*>(p)+off,&v,sizeof(v));}
static void arrange(IOMemoryMap &map,unsigned pipe,uint32_t htotal,Display13WM::PipeTiming kept,uint64_t hz,uint32_t held){
 memset(path,0,sizeof(path));memset(scaler,0,sizeof(scaler));memset(accessObject,0,sizeof(accessObject));
 sizes.clear();sizes[fb]=sizeof(fb);sizes[path]=sizeof(path);sizes[scaler]=sizeof(scaler);sizes[accessObject]=sizeof(accessObject);
 table[0x130/8]=reinterpret_cast<uintptr_t>(&writer);
 put(accessObject,0,static_cast<void*>(table));put(accessObject,0x60,0x1000000U);
 put(path,0x32C0,reinterpret_cast<OSObject*>(scaler));
 display13FBPower={};auto &c=display13FBPower;
 c.controller=reinterpret_cast<IOService*>(controller);c.map=&map;c.access=accessObject;
 if(pipe<3){c.pipe[pipe].wm[0].status=Display13WM::Status::Complete;c.pipe[pipe].wm[0].htotal=htotal;c.pipe[pipe].pipeTiming=kept;}
 map.reg(0x45404)=0xFC0F;
 for(unsigned p=0;p<3;++p)map.reg(0x45270+4*p)=0x1A0+p;
 if(pipe<3)map.reg(0x45270+4*pipe)=held;
 nullHz=hz;rateCalls=0;writes.clear();dropWrites=false;publications=0;published.clear();mapped=&map;
}
static Display13WM::Program run(unsigned pipe,uint64_t writerAddress=reinterpret_cast<uintptr_t>(&writer)){
 return adlnProgramPipeLineTime(reinterpret_cast<IOService*>(fb),reinterpret_cast<OSObject*>(path),pipe,
  reinterpret_cast<uintptr_t>(&rate),writerAddress);
}
static void untouched(IOMemoryMap &map,unsigned pipe){
 for(unsigned p=0;p<3;++p)if(p!=pipe)assert(map.reg(0x45270+4*p)==0x1A0+p);
}
int main(){
 using S=Display13WM::Status;using T=Display13WM::PipeTiming;
 static IOMemoryMap map;
 const T boot{2720,62,353516853},fromNative{2720,89,245500000},between{2720,71,310710937};
 for(unsigned pipe=0;pipe<3;++pipe){
  // Booted directly into the downscaled source: the register still holds the firmware value.
  arrange(map,pipe,2720,boot,353516853,89);auto r=run(pipe);
  assert(r.status==S::Complete && r.writes==1 && rateCalls==1 && writes.size()==1);
  assert(writes[0].reg==0x45270+4*pipe && writes[0].value==62 && map.reg(0x45270+4*pipe)==62);
  assert(publications==1 && published.size()==8 && published[1]==uint64_t(S::Complete) && published[2]==1 &&
   published[3]==0x45270+4*pipe && published[4]==62 && published[5]==62);
  untouched(map,pipe);
  // Entered from the native mode: calculated before the scaler took effect, already programmed natively.
  arrange(map,pipe,2720,fromNative,353516853,89);r=run(pipe);
  assert(r.status==S::Complete && !r.writes && writes.empty() && rateCalls==1 && !publications);untouched(map,pipe);
  // Between two downscaled sources, should the native underscan path not have run yet.
  arrange(map,pipe,2720,between,310710937,62);r=run(pipe);
  assert(r.status==S::Complete && writes.size()==1 && writes[0].value==71 && map.reg(0x45270+4*pipe)==71);untouched(map,pipe);
  // No record: the rate at this point decides, equal or not.
  arrange(map,pipe,2720,T{},245500000,89);r=run(pipe);
  assert(r.status==S::Complete && writes.empty());
  arrange(map,pipe,2720,T{},245500000,0);r=run(pipe);
  assert(r.status==S::Complete && writes.size()==1 && writes[0].value==89);untouched(map,pipe);
  // A record for another transcoder timing is not used.
  arrange(map,pipe,2200,boot,148500000,0);r=run(pipe);
  assert(r.status==S::Complete && writes.size()==1 && writes[0].value==119);
  // A write that does not stick is reported, with what was read.
  arrange(map,pipe,2720,boot,353516853,89);dropWrites=true;r=run(pipe);
  assert(r.status==S::Readback && writes.size()==1 && map.reg(0x45270+4*pipe)==89);
  assert(publications==1 && published[1]==uint64_t(S::Readback) && published[4]==62 && published[5]==89);
  // An unpowered pipe is not written.
  arrange(map,pipe,2720,boot,353516853,89);map.reg(0x45404)&=~(1U<<(10+2*pipe));r=run(pipe);
  assert(r.status==S::Power && writes.empty() && map.reg(0x45270+4*pipe)==89);
  // Refusals: nothing is calculated, nothing reaches the writer.
  for(unsigned refusal=0;refusal<9;++refusal){
   arrange(map,pipe,2720,boot,353516853,89);auto &c=display13FBPower;uint64_t address=reinterpret_cast<uintptr_t>(&writer);
   switch(refusal){
    case 0:c.pipe[pipe].wm[0].status=S::Unavailable;break;
    case 1:put(path,0x32C0,static_cast<OSObject*>(nullptr));break;
    case 2:sizes[scaler]=0x57;break;
    case 3:sizes[path]=0x372F;break;
    case 4:sizes[accessObject]=0x67;break;
    case 5:put(accessObject,0x60,0x800000U);break;
    case 6:table[0x130/8]=reinterpret_cast<uintptr_t>(&otherWriter);break;
    case 7:address=reinterpret_cast<uintptr_t>(&otherWriter);break;
    case 8:c.map=nullptr;break;
   }
   r=run(pipe,address);
   assert(r.status==S::Unavailable && !r.writes && writes.empty() && !rateCalls && !publications && map.reg(0x45270+4*pipe)==89);
   untouched(map,pipe);
  }
 }
 for(auto pipe:{3U,UINT32_MAX}){
  arrange(map,pipe,2720,boot,353516853,89);const auto r=run(pipe);
  assert(r.status==S::Unavailable && writes.empty() && !rateCalls && !publications);untouched(map,pipe);
 }
 arrange(map,0,2720,boot,353516853,89);
 assert(adlnProgramPipeLineTime(nullptr,reinterpret_cast<OSObject*>(path),0,reinterpret_cast<uintptr_t>(&rate),reinterpret_cast<uintptr_t>(&writer)).status==S::Unavailable);
 assert(adlnProgramPipeLineTime(reinterpret_cast<IOService*>(fb),nullptr,0,reinterpret_cast<uintptr_t>(&rate),reinterpret_cast<uintptr_t>(&writer)).status==S::Unavailable);
 assert(adlnProgramPipeLineTime(reinterpret_cast<IOService*>(fb),reinterpret_cast<OSObject*>(path),0,0,reinterpret_cast<uintptr_t>(&writer)).status==S::Unavailable);
 assert(adlnProgramPipeLineTime(reinterpret_cast<IOService*>(fb),reinterpret_cast<OSObject*>(path),0,reinterpret_cast<uintptr_t>(&rate),0).status==S::Unavailable);
 assert(writes.empty() && !rateCalls);
 puts("PASS production pipe line time: calculated value is programmed once through the native writer where the register differs, left alone where equal, per pipe; refusals, a dropped write and an unpowered pipe do not report completion or touch other registers");
}
'''.replace('BODY',body)
with tempfile.TemporaryDirectory(prefix='line-time-') as directory:
 p=Path(directory);(p/'test.cpp').write_text(code)
 subprocess.run(['clang++','-std=c++17','-O1','-fsanitize=address,undefined','-I'+str(root/'WhateverXe'),str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
