#!/usr/bin/env python

#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, OutputChangeEventArgs, SensorChangeEventArgs
from Phidgets.Devices.InterfaceKit import InterfaceKit

#Create an interfacekit object
try:
	interfaceKit = InterfaceKit()
except RuntimeError as e:
	print("Runtime Exception: %s" % e.details)
	print("Exiting....")
	exit(1)

#Information Display Function
def displayDeviceInfo():
	print("|------------|----------------------------------|--------------|------------|")
	print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
	print("|------------|----------------------------------|--------------|------------|")
	print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (interfaceKit.isAttached(), interfaceKit.getDeviceName(), interfaceKit.getSerialNum(), interfaceKit.getDeviceVersion()))
	print("|------------|----------------------------------|--------------|------------|")
	print("Number of Digital Inputs: %i" % (interfaceKit.getInputCount()))
	print("Number of Digital Outputs: %i" % (interfaceKit.getOutputCount()))
	print("Number of Sensor Inputs: %i" % (interfaceKit.getSensorCount()))

#Event Handler Callback Functions
def interfaceKitAttached(e):
	attached = e.device
	print("InterfaceKit %i Attached!" % (attached.getSerialNum()))

def interfaceKitDetached(e):
	detached = e.device
	print("InterfaceKit %i Detached!" % (detached.getSerialNum()))

def interfaceKitError(e):
	try:
		source = e.device
		print("InterfaceKit %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))

def interfaceKitInputChanged(e):
	source = e.device
	print("InterfaceKit %i: Input %i: %s" % (source.getSerialNum(), e.index, e.state))

def interfaceKitSensorChanged(e):
	source = e.device
	print("InterfaceKit %i: Sensor %i: %i" % (source.getSerialNum(), e.index, e.value))

def interfaceKitOutputChanged(e):
	source = e.device
	print("InterfaceKit %i: Output %i: %s" % (source.getSerialNum(), e.index, e.state))

def closePhidget():
	try:
		interfaceKit.closePhidget()
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		print("Exiting....")
		exit(1)

def phidgetsLauncher(input_changed_callback=interfaceKitInputChanged):
	#Main Program Code
	try:
		interfaceKit.setOnAttachHandler(interfaceKitAttached)
		interfaceKit.setOnDetachHandler(interfaceKitDetached)
		interfaceKit.setOnErrorhandler(interfaceKitError)
		interfaceKit.setOnInputChangeHandler(input_changed_callback)
		interfaceKit.setOnOutputChangeHandler(interfaceKitOutputChanged)
		interfaceKit.setOnSensorChangeHandler(interfaceKitSensorChanged)
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		print("Exiting....")
		exit(1)

	print("Opening phidget object....")

	try:
		interfaceKit.openPhidget()
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		print("Exiting....")
		exit(1)

	print("Waiting for attach....")

	try:
		interfaceKit.waitForAttach(10000)
	except PhidgetException as e:
		print("Phidget Exception %i: %s" % (e.code, e.details))
		try:
			interfaceKit.closePhidget()
		except PhidgetException as e:
			print("Phidget Exception %i: %s" % (e.code, e.details))
			print("Exiting....")
			exit(1)
		print("Exiting....")
		exit(1)
	else:
		displayDeviceInfo()

	# print("Setting the data rate for each sensor index to 4ms....")
	# for i in range(interfaceKit.getSensorCount()):
	#     try:
			
	#         interfaceKit.setDataRate(i, 4)
	#     except PhidgetException as e:
	#         print("Phidget Exception %i: %s" % (e.code, e.details))
