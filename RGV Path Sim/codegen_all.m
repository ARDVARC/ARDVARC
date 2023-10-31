cd RGV\
codegen getRGVstatesAtTimes.m -jit -config:mex
codegen makeRGVfromSeed.m -jit -config:mex
cd ..\

cd UAS\
codegen getUASstatesAtTimes.m -jit -config:mex
cd ..\