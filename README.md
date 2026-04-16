# SignalProcessing
Resolved the issue of noises from lora modules rssi values to estimate the distance
# Median Filter (The Edge Preserver): 
Instead of blurring data like a moving average, it sorts a small window of values and picks the middle one. This completely eliminates random noise spikes while keeping sharp, abrupt changes perfectly intact.

# Z-Transform (The System Analyzer): 
A mathematical tool that converts time-based signals into the frequency domain. Engineers use it to design and analyze linear filters to see exactly which noise frequencies will be blocked or passed.

# Monte Carlo / Particle Filters (The Probabilistic Tracker): 
Instead of one exact calculation, it uses thousands of random, probability-based "guesses" to track a signal. It excels at handling abrupt changes in complex systems because some "guesses" are always anticipating sudden jumps.
