list = ['Google', 'Runoob', 1997, 2000]
list1=[1,2,3,4]
print("origin",list)
list2=list + list1
for x in list1:
	print("end:",x)
	print("in index of list1 is :",list1.index(3))
print("length",len(list))
print("max",max(list1))
print("min",min(list1))
print("now list2:",list2)
del list[2]
print("now2",list)

