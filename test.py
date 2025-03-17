with open("t.txt", 'r') as f:
	d = f.read()
	try:
		data = list(map(int, d.split("\n")))
	except:
		data = list(map(float, d.split("\n")))

	print(data)
