function symMatFuncPath = getMatlabFunction(matFuncRootPath, matFuncName, inputVars, symPhysicalParams, symStates)
    symMatFuncPath = fullfile(matFuncRootPath, [matFuncName, '.m']);
    symInputVars = {symPhysicalParams, symStates};
    % matlabFunction(outputVars, 'File', symMatFuncPath, 'Vars', symInputVars, 'Outputs', outputVars, 'Sparse', true);
    matlabFunction(inputVars, 'File', symMatFuncPath, 'Vars', symInputVars);
end
