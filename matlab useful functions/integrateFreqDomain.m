function y = integrateFreqDomain(x, Fs, order)
%INTEGRATEFREQDOMAIN Integrate or Derivate a time-series via FFT.
%
%   y = integrateFreqDomain(x, Fs)              % default is 1st integration
%   y = integrateFreqDomain(x, Fs, order)       % order: -2 to +2
%
% INPUTS
%   x     – column- or row-vector with N samples of the signal in the time domain
%   Fs    – sampling rate  [Hz]
%   order – -2 → d²x/dt² (2nd deriv.)
%           -1 → dx/dt    (1st deriv.)
%            0 → x        (no operation)
%           +1 → ∫ x dt   (1st integ.)
%           +2 → ∬ x dt²  (2nd integ.)
%
% OUTPUT
%   y  – transformed signal (same size & orientation as x)
%
% Lucian – Jun-2025
% -------------------------------------------------------------------------

    if nargin < 3,  order = 1;  end
    if ~ismember(order, -2:2)
        error('order must be -2, -1, 0, +1, or +2.');
    end

    % --- FFT & frequency axis ------------------------------------------------
    x  = x(:);                    % force column for internal math
    N  = numel(x);
    X  = fft(x);                  % complex spectrum
    f  = (0:N-1).' * Fs / N;      % raw frequency bins [0, Fs)
    w  = 2*pi*(f - (f >= Fs/2)*Fs); % centered in 0, signed frequency (rad/s)

    % Avoid division by zero at DC
    w_nozero = w;
    w_nozero(w == 0) = Inf;

    % --- Frequency-domain transformation -------------------------------------
    switch order
        case 0
            Xout = X;
        case 1
            Xout = X ./ (1j*w_nozero);     % single integration
        case 2
            Xout = X ./ (-(w_nozero.^2));  % double integration
        case -1
            Xout = 1j*w .* X;              % single derivative
        case -2
            Xout = -(w.^2) .* X;           % double derivative
    end

    % Force zero DC component for integration
    if order > 0
        Xout(w == 0) = 0;
    end

    % --- IFFT back to time ---------------------------------------------------
    y  = real(ifft(Xout));
    if isrow(x),  y = y.';  end

    % --- PLOT: Positive Frequency Spectrum (linear) --------------------------
    N_half = floor(N/2) + 1;      % includes DC and Nyquist if even
    f_pos  = f(1:N_half);
    magXout = 2 * abs(Xout(1:N_half)) / N;

    figure;
    loglog(f_pos, magXout, 'LineWidth', 1.2);
    grid on; xlabel('Frequency [Hz]'); ylabel('Magnitude');
    title('Espectro');
    legend('Velocidade');
end
