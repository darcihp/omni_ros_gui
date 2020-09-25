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

class_names = ["Jan", "Fev", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dez"]

def plot_confusion_matrix(cm, classes, normalize=False, title='Confusion matrix', cmap=plt.cm.Blues):
	plt.imshow(cm, interpolation='nearest', cmap=cmap)
	plt.title(title)
	plt.colorbar()
	tick_marks = numpy.arange(len(classes))
	plt.xticks(tick_marks, classes, rotation=45)
	plt.yticks(tick_marks, classes)

	if normalize:
		cm = cm.astype('float') / cm.sum(axis=1)[:, numpy.newaxis]
		print("Normalized confusion matrix")
	else:
		print('Confusion matrix, without normalization')

	print(cm)

	thresh = cm.max() / 2.
	for i, j in itertools.product(range(cm.shape[0]), range(cm.shape[1])):
        	plt.text(j, i, "{:0.2f}".format(cm[i, j]), horizontalalignment="center", color="white" if cm[i, j] > thresh else "black")

	plt.tight_layout()
	plt.ylabel('True label')
	plt.xlabel('Predicted label')

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
	_svr = make_pipeline(StandardScaler(), LinearSVR(tol=1e-5))

	print ('Fitting SVM...')
	svr = _svr.fit(X_train, y_train)

	print ('Predicting...')
	# Predicao do classificador
	y_pred = svr.predict(X_test)

	# Determina acurácia
	accuracy = svr.score(X_test, y_test)

	# cria a matriz de confusao
	#cm = confusion_matrix(y_test, y_pred)

	#numpy.set_printoptions(precision=2)

	#plt.figure()
	#plot_confusion_matrix(cm, classes=class_names, normalize=True, title="SVM One-Versus-Rest: Xception - Data Augmentation")
	#plt.savefig(ml_lab3_path + "/xception/svm/yes_cm.png")

	print("Accuracy: %f" %(accuracy))
	#precision, recall, f1_score, support = precision_recall_fscore_support(y_test, y_pred, average='weighted')
	#_fout.write(str(accuracy) + " " + str(precision) + " " + str(recall) + " " + str(f1_score))
	_fout.write(str(accuracy))
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
		fout = open(omni_ros_gui_path + "/results/results", "w")

		svm(omni_ros_gui_path + "/out.svm", omni_ros_gui_path + "/out_test.svm", fout)

		#X, y = load_boston(return_X_y=True)
		#print(X.shape)
		#print (y)

		fout.close

		print("Done")

	except KeyboardInterrupt:
	        rospy.loginfo("Shutting down")

if __name__ == "__main__":
        main(sys.argv)


