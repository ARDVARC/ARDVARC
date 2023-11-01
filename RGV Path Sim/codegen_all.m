function codegen_all(enableAutoParallelization, enableJIT, enableProfiler)
    arguments
        enableAutoParallelization (1,1) logical = true;
        enableJIT (1,1) logical = false;
        enableProfiler (1,1) logical = false;
    end

    cfg = coder.config("mex");
    cfg.EnableAutoParallelization = enableAutoParallelization;
    cfg.EnableJIT = enableJIT;
    cfg.EnableMexProfiling = enableProfiler;

    cd RGV\
    codegen getRGVstatesAtTimes.m -config cfg
    codegen makeRGVfromSeed.m -config cfg
    cd ..\
    
    cd UAS\
    codegen getUASstatesAtTimes.m -config cfg
    cd ..\
end