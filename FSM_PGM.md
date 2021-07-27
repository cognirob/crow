# FSM and PGM

* Markov chain (first order Markov process)  
    - FSM is a special case/implementation of MC

* HMM ~ MC where the state is not directly observable  
    - states q
    - outputs z (x in literature)
    - transition prob: p(q(t) | q(t-1))
    - emission (on trans) prob: p(z(t) | q(t-1), q(t))
    - emission (on state) prob: p(z(t) | q(t))
* IOHMM - input/output HMM (Bengio and Frasconi, 1995)  
    - a modification of HMM
    - states q
    - inputs y
    - outputs z
    - transition prob: p(q(t) | y(t), q(t-1))
    - emission (on trans) prob: p(z(t) | q(t), q(t-1), y(t), y(t-1))
    - emission (on state) prob: p(z(t) | q(t), y(t))