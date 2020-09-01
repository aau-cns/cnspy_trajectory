import os
from Trajectory import Trajectory
from csv2dataframe.PoseWithCov2DataFrame import PoseWithCov2DataFrame
from csv2dataframe.CSV2DataFrame import CSV2DataFrame
from ros_csv_formats.CSVFormat import CSVFormat


class TrajectoryEstimated(Trajectory):
    Sigma_p_vec = None
    Sigma_q_vec = None

    def __init__(self, t_vec=None, p_vec=None, q_vec=None, Sigma_p_vec=None, Sigma_q_vec=None, df=None):
        Trajectory.__init__(self, t_vec=t_vec, p_vec=p_vec, q_vec=q_vec)
        self.Sigma_p_vec = Sigma_p_vec
        self.Sigma_q_vec = Sigma_q_vec

        if df is not None:
            self.load_from_DataFrame(df)

    def load_from_CSV(self, filename):
        if not os.path.isfile(filename):
            print("Trajectory: could not find file %s" % os.path.abspath(filename))
            return False

        df = CSV2DataFrame.load_CSV(filename=filename, fmt=CSVFormat.PoseWithCov)
        self.load_from_DataFrame(df)
        return True

    def load_from_DataFrame(self, df):
        self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec, self.Sigma_q_vec = PoseWithCov2DataFrame.DataFrame_to_TPQCov(
            data_frame=df)

    def to_DataFrame(self):
        return PoseWithCov2DataFrame.TPQCov_to_DataFrame(self.t_vec, self.p_vec, self.q_vec, self.Sigma_p_vec,
                                                         self.Sigma_q_vec)

    def save_to_CSV(self, filename):
        if self.is_empty():
            return False
        df = self.to_DataFrame()
        CSV2DataFrame.save_CSV(df, filename=filename, fmt=CSVFormat.PoseWithCov)
        return True


########################################################################################################################
#################################################### T E S T ###########################################################
########################################################################################################################
import unittest
import time


class TrajectoryEstimated_Test(unittest.TestCase):
    start_time = None

    def start(self):
        self.start_time = time.time()

    def stop(self):
        print "Process time: " + str((time.time() - self.start_time))

    def load_(self):
        print('loading...')
        fn = '../sample_data/ID1-pose-est-cov.csv'
        obj = TrajectoryEstimated()
        obj.load_from_CSV(filename=fn)
        return obj

    def test_load_trajectory_from_CSV(self):
        self.start()
        obj = self.load_()
        # self.assertTrue(obj.data_loaded)
        self.stop()

        print(obj.Sigma_p_vec[1000])
        print(obj.Sigma_p_vec[1000])
        self.start()
        obj.save_to_CSV(filename='../test/ID1-pose-est-cov-copy.csv')
        self.stop()


if __name__ == "__main__":
    unittest.main()
