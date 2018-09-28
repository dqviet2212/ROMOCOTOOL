function kChains = getKinematicChainDef()
    %% Kinematic chain 1
    kChains(1).joint(1).axis = [0; 1; 0]; kChains(1).joint(1).type = 'revolute';
    kChains(1).joint(2).axis = [0; 1; 0]; kChains(1).joint(2).type = 'revolute';
    kChains(1).joint(3).axis = [0; 1; 0]; kChains(1).joint(3).type = 'revolute';
end
