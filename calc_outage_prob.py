import numpy as np


alpha = 1
P = 1
Rth = .2
sigma = 1e-32

def SNR(K: int):
    min_h_square = float("inf")

    for _ in range(K):
        a, b = np.random.normal(0, alpha, 2)
        h_square = a**2 + b**2
        if h_square < min_h_square:
            min_h_square = h_square
    
    return P*min_h_square/(sigma**2)

def transmission_data_rate(K: int):
    return np.log2(1 + SNR(K)) / (K+1)

def outage_probability(K: int):
    x = np.log(2) * Rth * (K+1)
    numerator = sigma*sigma*(K+1)*(1-np.exp(x))
    denominator = alpha * P

    return 1 - np.exp(numerator/denominator)


def main():
    print(outage_probability(1000))

if __name__ == "__main__":
    main()


