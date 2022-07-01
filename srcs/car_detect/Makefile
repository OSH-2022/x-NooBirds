.SUFFIXES:.c .o

OBJS_DIR = obj

SRCS = test.cpp detect_const.cpp timer.cpp coordinate.cpp process_predict.cpp

OBJS = $(SRCS:%.cpp=$(OBJS_DIR)/%.o)

CC = g++
CC_FLAGS = -g -Wall -O3 `pkg-config --libs --cflags opencv4` -std=c++17 -fsanitize=address
INCLUDE =
LINK = `pkg-config --libs --cflags opencv4`

.PHONY:default clean distclean test

default:test $(OBJS) Makefile
	$(CC) $(CC_FLAGS) $(OBJS) -o demo $(LINK)
	@echo "------ build ok ------"

test:
ifneq ($(OBJS_DIR), $(wildcard $(OBJS_DIR)))
	@mkdir -p $(OBJS_DIR)
endif

$(OBJS): $(OBJS_DIR)/%.o : %.cpp
	$(CC) $(CC_FLAGS) $(INCLUDE) -c $< -o $@ $(LINK)

clean:
	@rm -rf $(OBJS_DIR)/*.o *.o
	@echo "---- clean ok ----"

distclean:clean
	@rm -rf *.exe demo $(OBJS_DIR)

