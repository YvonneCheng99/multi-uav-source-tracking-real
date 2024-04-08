from scipy.optimize import minimize
import numpy as np


class GPR:

    def __init__(self, optimize=True):
        self.is_fit = False
        self.train_X, self.train_y = None, None
        self.params = {"l": 0.5, "sigma_f": 0.6, "a": -46, "n": 2, "w_1": 0.0, "w_2": 0.0, "v_1": 1.0, "v_2": 1.0}
        self.noise_sigma = 0.8
        self.optimize = optimize
        self.Kff_inv = None

    def fit(self, X, y):
        # store train data
        self.train_X = np.asarray(X)
        self.train_y = np.asarray(y)
        # print("fit----------------------")
        # print(self.train_X)
        # print(self.train_y)

        # hyper parameters optimization
        def negative_log_likelihood_loss(params):
            self.params["l"], self.params["a"], self.params["n"], self.params["w_1"], self.params["w_2"],\
                 self.params["v_1"], self.params["v_2"] \
                     = params[0], params[1], params[2], params[3], params[4], params[5], params[6]
            Kyy = self.kernel(self.train_X, self.train_X) + self.noise_sigma**2 * np.eye(len(self.train_X))
            MY = self.mean(self.train_X)
            return (self.train_y.T-MY.T).dot(np.linalg.inv(Kyy)).dot(self.train_y - MY) + np.linalg.slogdet(Kyy)[1]
            # return 0.5 * self.train_y.T.dot(np.linalg.inv(Kyy)).dot(self.train_y) + 0.5 * np.linalg.slogdet(Kyy)[
            #     1] + 0.5 * len(self.train_X) * np.log(2 * np.pi)

        if self.optimize:
            res = minimize(negative_log_likelihood_loss,
                           [self.params["l"], self.params["a"], self.params["n"], self.params["w_1"], 
                           self.params["w_2"], self.params["v_1"], self.params["v_2"]],
                           bounds=((1e-2, 1e2), (-1e2, -1e-2), (1e-2, 1e2), (-1e1, 1e1), 
                           (-1e1, 1e1), (-1, 1), (-1, 1)),
                           method='L-BFGS-B')
            self.params["l"], self.params["a"], self.params["n"], self.params["w_1"], \
                self.params["w_2"], self.params["v_1"], self.params["v_2"] = \
                    res.x[0], res.x[1], res.x[2], res.x[3], res.x[4], res.x[5], res.x[6]
            print("params-----------------")
            print(res.x[0])
            print(res.x[1])
            print(res.x[2])
            print(res.x[3])
            print(res.x[4])
            print(res.x[5])
            print(res.x[6])

        self.is_fit = True

    def predict(self, X):
        if not self.is_fit:
            print("model not fit yet.")
            return

        X = np.asarray(X)
        # print("predict---------------------------")
        # print(X)
        # print(self.train_X)
        Kff = self.kernel(self.train_X, self.train_X)  # (N, N)
        Kyy = self.kernel(X, X)  # (k, k)
        Kfy = self.kernel(self.train_X, X)  # (N, k)
        My = self.mean(X)
        MY = self.mean(self.train_X)
        Kff_inv = np.linalg.inv(Kff + 1e-8 * np.eye(len(self.train_X)))  # (N, N)
        self.Kff_inv = Kff_inv

        mu = My + Kfy.T.dot(Kff_inv).dot(self.train_y - MY)
        cov = Kyy - Kfy.T.dot(Kff_inv).dot(Kfy)

        return mu, cov

    def kernel(self, x1, x2):
        # print("kernel------------------")
        # print(x1)
        # print(x2)
        x1 = np.atleast_2d(x1)
        x2 = np.atleast_2d(x2)
        dist_matrix = np.sum(x1 ** 2, 1).reshape(-1, 1) + np.sum(x2 ** 2, 1) - 2 * np.dot(x1, x2.T)
        # strength = self.params["a"] - self.params["n"] * 10 * np.log10(np.sqrt(dist_matrix))
        # return self.params["sigma_f"] ** 2 * np.exp(-0.5 / self.params["l"] ** 2 * strength)
        return self.params["sigma_f"] ** 2 * np.exp(-0.5 / self.params["l"] ** 2 * dist_matrix)

    def mean(self, X):
        s_1 = []
        s_2 = []
        t = []
        for i in X:
            s_1.append(i[0])
            s_2.append(i[1])
            t.append(i[2])
        
        # print("mean-------------------")
        # print(X)
        # print(s_1)
        # print(s_2)
        # print(t)
        l = np.asarray([s_1 + np.dot(self.params["v_1"], t), s_2 + np.dot(self.params["v_2"], t)])
        # print(np.transpose(l))
        dist = np.linalg.norm(np.transpose(l) \
            - np.asarray([self.params["w_1"], self.params["w_2"]]), axis=1)
        return self.params["a"] - self.params["n"] * 10 * np.log10(dist)

    def get_Kff_inv(self):
        return self.Kff_inv