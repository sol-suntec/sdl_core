%language=C++
%struct-type
%define lookup-function-name getPointer
%define class-name ButtonEventMode_intHash
%omit-struct-type
%readonly-tables
%compare-strncmp
%compare-lengths

struct NsAppLinkRPCV1::PerfectHashTable
{
  const char *name;
  unsigned int idx;
};

%%
BUTTONUP,0
BUTTONDOWN,1
