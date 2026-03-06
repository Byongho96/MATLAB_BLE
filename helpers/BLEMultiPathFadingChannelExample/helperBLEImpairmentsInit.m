function init = helperBLEImpairmentsInit(phyMode,sps)
%helperBLEImpairmentsInit Initialize RF impairment parameters
%
%   INIT = helperBLEImpairmentsInit(PHYMODE,SPS) outputs a structure, INIT,
%   which contains the front-end impairment parameters for the given
%   PHYMODE and SPS. PHYMODE is a character vector or string specifying the
%   PHY on which decoding is performed. It must be one of the following:
%   "LE1M","LE2M","LE500K","LE125K". SPS denotes the number of samples per
%   symbol and must be a positive integer.
%
%   See also bleWaveformGenerator, bleIdealReceiver.

%   Copyright 2018-2022 The MathWorks, Inc.

% Initialize parameters Define sample rate based on the PHY mode
symbolRate = 1e6;
sampleRate = symbolRate*(1+(phyMode=="LE2M"))*sps;

% Initialize frequency and phase offset system object
init.pfo = comm.PhaseFrequencyOffset(SampleRate=sampleRate);

% Initialize variable timing offset system object
init.varDelay = dsp.VariableFractionalDelay;

% Initialize phase noise system object
init.pnoise = comm.PhaseNoise(Level=[-130 -136],FrequencyOffset=[1e4 1e5],SampleRate=sampleRate);
end