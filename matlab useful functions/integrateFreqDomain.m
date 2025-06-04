function y = integrateFreqDomain(x, Fs, order)
%INTEGRATEFREQDOMAIN Integrate (or double-integrate) a time-series via FFT.
%
%   y = integrateFreqDomain(x, Fs)           % one integration
%   y = integrateFreqDomain(x, Fs, order)    % 'order' = 1 (vel) or 2 (disp)
%
% INPUTS
%   x     – column- or row-vector with N samples of the signal in the time domain
%   Fs    – sampling rate  [Hz]
%   order – 1 →  ∫ x dt  (default)      (e.g. acc → vel)
%            2 → ∬ x dt² (double int.)  (e.g. acc → disp)
%
% OUTPUT
%   y  – integrated signal (same size & orientation as x)
%
% NOTES
% • Integration constant is forced to zero by setting the DC bin to 0.
% • No additional filtering is applied; if your signal contains low-frequency
%   drift you might want to high-pass x before calling this function.
% • Works for both real and complex inputs.
%
% Lucian & Bruna – Jun-2025
% -------------------------------------------------------------------------

    if nargin < 3,  order = 1;  end
    if order ~= 1 && order ~= 2
        error('order must be 1 (single) or 2 (double) integration.');
    end

    % --- FFT & frequency axis ------------------------------------------------
    x  = x(:);                    % force column for internal math
    N  = numel(x);
    X  = fft(x);                  % complex spectrum, 0…N-1
    k  = (0:N-1).';               % integer bins

    % Build symmetric frequency vector f = [0 … +Fs/2 … -Fs/2 … -Fs/N]*Hz
    f  = (k - (k >= N/2).*N) * Fs / N;      % Hz
    w  = 2*pi*f;                             % rad/s  (signed!)

    % --- Integrate in the frequency domain ----------------------------------
    w(w==0) = Inf;     % avoid division by zero at DC
    switch order
        case 1
            Xint          =  X ./ (1j*w);      % single integration
        case 2
            Xint          =  X ./ (-(w.^2));   % double integration
    end
    Xint(w == Inf) = 0;  % force zero DC component (integration constant)

    % --- IFFT back to time ---------------------------------------------------
    y  = real(ifft(Xint));

    % restore original orientation
    if isrow(x),  y = y.';  end
end
