from rospy_wrapper import ROSBagSource

def main():
	source = ROSBagSource('features.bag')
	for msg, t in source:
		

if __name__ == '__main__':
	main()
