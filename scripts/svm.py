#!/usr/bin/env python3
# -*- encoding: iso-8859-1 -*-
import sys
import numpy
import rospy
import roslib
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
from sklearn.datasets import load_svmlight_file
from sklearn import preprocessing
import pylab as pl
import rospkg
import os
from sklearn.metrics import precision_recall_fscore_support
import datetime
import matplotlib.pyplot as plt
import itertools

from sklearn.svm import SVR
from sklearn.svm import LinearSVR
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_squared_error
from sklearn.svm import SVR


def svm(data_train, data_test, _fout):

	print ("Loading data...")
	print (data_train)
	print (data_test)

	X_train, y_train = load_svmlight_file(data_train)
	X_test, y_test = load_svmlight_file(data_test)

	X_train = X_train.toarray()
	X_test = X_test.toarray()

	#print("Normalizing...")
	# Fazer a normalizacao dos dados #######
	#scaler = preprocessing.MinMaxScaler()
	#X_train = scaler.fit_transform(X_train)
	#X_test = scaler.fit_transform(X_test)

	# Cria Regressor
	#_linearsvr = make_pipeline(StandardScaler(), LinearSVR(tol=1e-5))
	#print ('Fitting LinearSVR...')
	#linearsvr = _linearsvr.fit(X_train, y_train)
	#print ('Predicting...')
	# Predicao do classificador
	#y_pred = linearsvr.predict(X_test)
	#mse = mean_squared_error(y_test, y_pred)
	#print("The mean squared error (MSE) on test set: {:.4f}".format(mse))


	_svr = make_pipeline(StandardScaler(), SVR(C=1.0, epsilon=0.2))
	print ('Fitting SVR...')
	svr = _svr.fit(X_train, y_train)
	print ('Predicting...')
	# Predicao do classificador
	y_pred = svr.predict(X_test)
	mse = mean_squared_error(y_test, y_pred)
	print("The mean squared error (MSE) on test set: {:.4f}".format(mse))

	_fout.write(str(mse))
	_fout.write("\n")

def main(args):

	try:
		rospy.init_node('n_omni_ros_gui_svm', anonymous=True)
		rospack = rospkg.RosPack()

		#Caminho do package
		global omni_ros_gui_path
		omni_ros_gui_path = rospack.get_path("omni_ros_gui")
		omni_ros_gui_path += "/scripts"

		#Abre arquivo para escrita de resultados
		fout = open(omni_ros_gui_path + "/results/results_theta", "w")

		svm(omni_ros_gui_path + "/out_treino_theta.svm", omni_ros_gui_path + "/out_teste_theta.svm", fout)

		#X, y = load_boston(return_X_y=True)
		#print(X.shape)
		#print (y)

		fout.close

		print("Done")

	except KeyboardInterrupt:
	        rospy.loginfo("Shutting down")

if __name__ == "__main__":
        main(sys.argv)


