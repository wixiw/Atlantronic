obj-linux-test_segment_intersection += kernel/math/segment_intersection.o
obj-linux-test_segment_intersection += linux/test/math/test_segment_intersection.o
lib-linux-test_segment_intersection += -lm
bin-linux += test_segment_intersection

obj-linux-test_trigo += kernel/math/fx_math.o
obj-linux-test_trigo += linux/test/math/test_trigo.o
lib-linux-test_trigo += -lm
bin-linux += test_trigo

obj-linux-test_poly7 += kernel/math/poly7.o
obj-linux-test_poly7 += linux/test/math/test_poly7.o
lib-linux-test_poly7 += -lm
bin-linux += test_poly7