import math
import random
import scipy.stats

import Gaussian_progress_dynamic as gp
from GPy.util.linalg import pdinv
from scipy.stats import gaussian_kde
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy.stats import multivariate_normal
import numpy as np
import parameters as pm
from KDEpy import FFTKDE


class ADS:
    times_threshold_value = pm.times_threshold_value  # 两个阶段划分的阈值
    distance_1 = pm.distance_1  # exploration阶段的移动步长
    distance_2 = pm.distance_2  # exploitation阶段的移动步长
    candidates_number = pm.candidates_number  # 候选点的数量
    left_boundary = pm.left_boundary
    right_boundary = pm.right_boundary

    def __init__(self, X, Y):
        self.dim = 3
        d1 = np.arange(self.left_boundary, self.right_boundary, 0.1)
        d2 = np.arange(self.left_boundary, self.right_boundary, 0.1)
        d1 = np.round(d1, 1)
        d2 = np.round(d2, 1)
        d1, d2 = np.meshgrid(d1, d2)
        self.grid_point_list = [[d1, d2] for d1, d2 in zip(d1.ravel(), d2.ravel())]
        X = np.array(X)
        Y = np.array(Y)
        # ker = GPy.kern.RBF(input_dim=self.dim, ARD=True)
        # self.model = GPy.models.GPRegression(X=X, Y=Y, kernel=ker)

    # 根据候选点的数量计算出候选点的角度列表
    def calculate_candidates_points_angle(self):
        candidates_angle_list = []
        random_angel = random.uniform(0.0, math.pi / 12)
        for i in range(0, self.candidates_number):
            candidates_angle_list.append(math.pi * 2.0 / self.candidates_number * i + random_angel)
            # candidates_angle_list.append(math.pi * i / 2.0)
        return candidates_angle_list

    # 计算出当前位置的候选节点列表
    def calculate_candidates_points(self, current_position, candidates_angle_list, time_track, step_dis):
        candidates_list = []
        for i in range(0, self.candidates_number):
            candidates_x = current_position[0] + step_dis * math.sin(candidates_angle_list[i])
            candidates_y = current_position[1] + step_dis * math.cos(candidates_angle_list[i])
            if self.right_boundary >= candidates_x >= self.left_boundary and self.right_boundary >= candidates_y >= self.left_boundary:
                candidates_list.append([candidates_x, candidates_y, time_track])
        return candidates_list

    '''
    计算下一步的位置
    calculate_times: 飞行次数
    current_position: 当前位置
    time_track: 要计算的时刻
    step_dis: 飞行步长
    max_angle: 预测出的最强位置与当前位置的方向角
    '''
    def next_step(self, calculate_times, current_position, Kff_inv, time_track, train_X, cov, step_dis, max_angle):
        candidates_angle_list = self.calculate_candidates_points_angle()
        candidates_list = self.calculate_candidates_points(current_position, candidates_angle_list, time_track, step_dis)
        print('#########', end='')
        print(calculate_times, end='')
        print(':', end='')
        print(step_dis, end='')
        print('candidates_list', end='')
        print(candidates_list)
        candidate_pos = []

        for angle in candidates_angle_list:
            if abs(angle - max_angle) < math.pi / 2:
                candidates_x = current_position[0] + step_dis * math.sin(angle)
                candidates_y = current_position[1] + step_dis * math.cos(angle)
                if self.right_boundary >= candidates_x >= self.left_boundary and self.right_boundary>= candidates_y >= self.left_boundary:
                    candidate_pos.append([candidates_x, candidates_y, time_track])

        index = self.ALC(train_X, candidate_pos, Kff_inv, time_track, cov)
        res = candidate_pos[index]
        return res

    def get_reference_points_list(self, time_track):
        reference_d1 = np.arange(self.left_boundary, self.right_boundary, 0.1)
        reference_d2 = np.arange(self.left_boundary, self.right_boundary, 0.1)
        reference_d1, reference_d2 = np.meshgrid(reference_d1, reference_d2)
        t_list = [time_track] * len(reference_d1) * len(reference_d2)
        reference_points_list = [[d1, d2, d3] for d1, d2, d3 in zip(reference_d1.ravel(), reference_d2.ravel(), t_list)]
        return reference_points_list

    def get_index(self, point):
        x = round((point[0]-self.left_boundary)/0.1)
        y = round((point[1]-self.left_boundary)/0.1)
        return [x, y]

    # 返回值为ALC方差最大的候选点的索引
    def ALC(self, train_X, candidates_list, time_track, cov):
        gpr = gp.GPR(optimize=True)
        Kff = gpr.kernel(train_X, train_X)
        Kff_inv = np.linalg.inv(Kff + 1e-8 * np.eye(len(train_X)))
        # 参考点列表
        reference_points_list = self.get_reference_points_list(time_track)
        # 候选点列表
        # candidates_angle_list = self.calculate_candidates_points_angle()
        # candidates_list = self.calculate_candidates_points(current_position, candidates_angle_list)
        # 直接用gp.kernel计算ALC
        # 对于每一个候选点，计算候选点的ALC公式的值，计算时需要计算每个参考点对应的值，将所有参考点的结果加起来，作为每个候选点的值
        train_data_num = len(train_X)
        ALC_list = [0.0 for i in range(len(candidates_list))]
        # ALC_list = [0.0 for i in range(len(candidates_list))]*len(candidates_list)
        for i in range(0, len(candidates_list)):
            candidate = candidates_list[i]
            index_c = self.get_index(candidate)
            sigma_2_xc = cov[index_c[0], index_c[1]]
            # print('-------sigma_x_xc------', end='')
            # print(sigma_2_xc)
            m = []  # the N-vector of covariances between the present training data points and the query candidate
            for j in range(0, train_data_num):
                train_data = train_X[j]
                # print('test-------')
                # print(train_data)
                # print(candidate)
                m.append(gpr.kernel(np.asarray(train_data), np.asarray(candidate))[0])
            m = np.atleast_2d(m)
            C_N_inv = Kff_inv
            C_xx = gpr.kernel(np.asarray(candidate), np.asarray(candidate))  # x代表候选点
            # 对于每一个参考点
            for j in range(0, len(reference_points_list)):
                k_N = []  # the vector of covariances between the training data and a reference data point
                reference_point = reference_points_list[j]
                '''
                index_r = self.get_index(reference_point)
                sigma_2_xr = cov[index_r[0], cov[index_r[1]]]
                '''
                '''
                for k in range(0, train_data_num):
                    train_data = train_X[k]
                    # print('k------')
                    # print(gpr.kernel(train_data, reference_point)[0])
                    k_N.append(gpr.kernel(train_data, reference_point)[0])
                C_xr = gpr.kernel(candidate, reference_point)  # r代表参考点
                k_N = np.atleast_2d(k_N)
                k_N = k_N.T
                # print(k_N)
                # print(C_N_inv)
                # print(m)
                molecule = (np.dot(np.dot(k_N, C_N_inv), m)-C_xr)  # 分子
                denominator = C_xx - np.dot(np.dot(m.T, C_N_inv), m)  # 分母
                delta_2 = molecule*molecule/denominator
                '''
                k_xc_xr = gpr.kernel(candidate, reference_point)
                k_xc_X = gpr.kernel(candidate, train_X)
                k_X_xr = gpr.kernel(train_X, reference_point)
                cov_x_x_star = k_xc_xr - np.dot(np.dot(k_xc_X, Kff_inv), k_X_xr)
                ALC_list[i] = ALC_list[i] + cov_x_x_star * cov_x_x_star
            ALC_list[i] = 1/sigma_2_xc*ALC_list[i]
        max_ALC = ALC_list[0]
        max_ALC_index = 0
        for i in range(0, len(ALC_list)):
            if ALC_list[i] > max_ALC:
                max_ALC = ALC_list[i]
                max_ALC_index = i
        return max_ALC_index

    # 带似然比的采集函数，用于探索极端区域，应用于我们的环境中为源周围的位置
    def likelihood_function(self, candidates_list, train_X, train_y, mu, cov, time_track, Kff_inv):
        # print('likelihood')
        # print(mu)
        # print(cov)
        # 参考点列表
        reference_points_list = self.get_reference_points_list(time_track)
        # reference_d1, reference_d2 = np.meshgrid(reference_d1, reference_d2)
        # reference_points_list = [[d1, d2] for d1, d2 in zip(reference_d1.ravel(), reference_d2.ravel())]
        # for i in range(0, len(reference_points_list)):
        #     reference_points_list[i].append(time_track)
        likelihood_list = [0.0 for i in range(len(candidates_list))]
        gpr = gp.GPR(optimize=True)
        mu_of_reference = []
        for reference_point in reference_points_list:
            mu_of_reference.append(self.get_rssi_by_coor(reference_point[0], reference_point[1], mu))
        kde = gaussian_kde(mu_of_reference)
        importance_density = kde.evaluate(mu_of_reference)
        importance_density /= np.sum(importance_density)
        for i in range(0, len(candidates_list)):
            candidate = candidates_list[i]
            var = self.gp_variance(candidate, train_X, noise_variance=0.6)
            for j in range(0, len(reference_points_list)):
                reference_point = reference_points_list[j]
                k_xc_xr = gpr.kernel(candidate, reference_point)
                k_xc_X = gpr.kernel(candidate, train_X)
                k_X_xr = gpr.kernel(train_X, reference_point)
                cov_x_x_star = k_xc_xr - np.dot(np.dot(k_xc_X, Kff_inv), k_X_xr)
                # cov_x_x_star = gpr.kernel(candidate, reference_point)
                fy = importance_density[j]
                # pdf = self.pdf(reference_point, mu, cov)
                # print(pdf)
                likelihood_ratio_ = self.likelihood_ratio(reference_point, train_X, train_y, fy)
                likelihood_list[i] = likelihood_list[i] + cov_x_x_star * cov_x_x_star * likelihood_ratio_
            likelihood_list[i] = 1/var*likelihood_list[i]

        min_likelihood = likelihood_list[0]
        min_likelihood_index = 0
        for i in range(0, len(likelihood_list)):
            if likelihood_list[i] < min_likelihood:
                min_likelihood = likelihood_list[i]
                min_likelihood_index = i
        return min_likelihood_index

    def pdf(self, x_star, mu, cov):
        epsilon = 1e-6  # 或者其他合适的小正数
        cov_matrix_stable = cov + epsilon * np.eye(len(cov))
        mvn = multivariate_normal(mean=mu, cov=cov_matrix_stable)
        # 需要把x_star转换成第几个点
        x_star_index = self.get_index_by_coor(x_star[0], x_star[1])
        pdf_value = mvn.pdf(np.array(1000))
        return pdf_value

    def kernel_matrices(self, x_star, X_train, noise_variance):
        gpr = gp.GPR(optimize=True)
        K = gpr.kernel(X_train, X_train)
        K_inv = np.linalg.inv(K + noise_variance * np.eye(len(X_train)))
        k_star = gpr.kernel(X_train, x_star)
        k_star_star = gpr.kernel(x_star, x_star)
        return K, K_inv, k_star, k_star_star

    def gp_variance(self, x_star, X_train, noise_variance):
        K, K_inv, k_star, k_star_star = self.kernel_matrices(x_star, X_train, noise_variance)

        # Posterior variance
        variance_post = k_star_star - k_star.T @ K_inv @ k_star
        return variance_post

    def likelihood_ratio(self, x, train_X, train_y, fy):
        x = np.atleast_2d(x)
        # fx = self.pdf(x, mu, cov)
        # fx = self.posterior_pdf(x, mu, cov)
        fx = self.gp_posterior_pdf(x, train_X, train_y, noise_variance=0.6)
        w = fx/fy
        return w

    def gp_posterior_pdf(selrf, x_star, X_train, y_train, noise_variance):
        gpr = gp.GPR(optimize=True)
        K = gpr.kernel(X_train, X_train)
        K_inv = np.linalg.inv(K + noise_variance * np.eye(len(X_train)))
        k_star = gpr.kernel(X_train, x_star)
        k_star_star = gpr.kernel(x_star, x_star)

        # Posterior mean
        mean_post = k_star.T @ K_inv @ y_train

        # Posterior covariance
        cov_post = k_star_star - k_star.T @ K_inv @ k_star

        # Multivariate normal distribution for posterior
        # print(mean_post.flatten())
        posterior_dist = multivariate_normal(mean=mean_post.flatten(), cov=cov_post)

        # Evaluate PDF at the given point
        pdf_value = posterior_dist.pdf(k_star.flatten())

        return pdf_value[0]
    '''
    def posterior_pdf(self, x_star, mu, cov):
        posterior_dist = multivariate_normal(mean=np.array(mu), cov=np.array(cov))
        # Evaluate PDF at the given point
        pdf_value = posterior_dist.pdf(x_star)
        return pdf_value
    '''

    def get_index_by_coor(self, x, y):
        coordinate_approximate = [round(x, 1), round(y, 1)]
        i = self.grid_point_list.index(coordinate_approximate)
        return i

    def get_rssi_by_coor(self, x, y, mu):
        coordinate_approximate = [round(x, 1), round(y, 1)]
        i = self.grid_point_list.index(coordinate_approximate)
        return mu[i]

    def custom_KDE(self, data, weights=None, bw=None):
        data = data.flatten()
        if bw is None:
            try:
                sc = scipy.stats.gaussian_kde(data, weights=weights)
                bw = np.sqrt(sc.covariance).flatten()
            except:
                bw = 1.0
            if bw < 1e-8:
                bw = 1.0
        return FFTKDE(bw=bw).fit(data, weights)
    '''
    def pdf(self, x, mu, cov):
        d = x - mu
        inv, _, _, ld = pdinv(cov)
        constant = -0.5*(self.dim * np.log(2*np.pi)+ld)
        lnpdf = constant - 0.5 * np.sum(d * np.dot(d, inv), 1)
        return np.exp(lnpdf)
    '''