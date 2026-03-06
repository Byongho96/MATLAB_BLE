function txImpairedWfm = helperBLEImpairmentsAddition(txWaveform,init)
%helperBLEImpairmentsAddition Adds RF impairments to the Bluetooth LE
%waveform
%
%   TXIMPAIREDWFM = helperBLEImpairmentsAddition(TXWAVEFORM,INIT) adds RF
%   impairments to the transmitted waveform, TXWAVEFORM, based on INIT and
%   outputs the impaired waveform, TXIMPAIREDWFM. INIT is a structure which
%   contains the parameters corresponding to DC, frequency, phase offsets,
%   timing drift, and phase noise. The following RF impairments are added
%   to the TXWAVEFORM.
%   * DC offset
%   * Carrier frequency offset
%   * Carrier phase offset
%   * Timing drift
%   * Phase noise
%
%   See also bleWaveformGenerator, bleIdealReceiver.

%   Copyright 2018-2024 The MathWorks, Inc.

% Add frequency and phase offset
txWfmFreqPhaseOffset = init.pfo(txWaveform);

% Add timing drift
if isa(txWfmFreqPhaseOffset,"single")
    init.vdelay = single(init.vdelay);
end
txWfmTimeOffset = init.varDelay(txWfmFreqPhaseOffset,init.vdelay);

% Add DC offset
dcValue = (init.dc/100)*max(txWfmTimeOffset);
txWfmDC = txWfmTimeOffset + dcValue;

% Add phase noise
txImpairedWfm = init.pnoise(txWfmDC);

% Release the System objects
release(init.pfo);

end