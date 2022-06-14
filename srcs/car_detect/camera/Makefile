.SUFFIXES:.c .o

OBJS_DIR = obj

SRCS = main.cpp camera.cpp bmp.cpp timer.cpp detect.cpp

OBJS = $(SRCS:%.cpp=$(OBJS_DIR)/%.o)

CC = g++
CC_FLAGS = -g -Wall -O3
INCLUDE = 
LINK = 

.PHONY:default clean distclean test
	
default:test $(OBJS) Makefile
	$(CC) $(LINK) $(CC_FLAGS) $(OBJS) -o demo
	@echo "------ build ok ------"
	
test: 
ifneq ($(OBJS_DIR), $(wildcard $(OBJS_DIR)))
	@mkdir -p $(OBJS_DIR)
endif

$(OBJS): $(OBJS_DIR)/%.o : %.cpp
	$(CC) $(LINK) $(CC_FLAGS) $(INCLUDE) -c $< -o $@

clean:
	@rm -rf $(OBJS_DIR)/*.o *.o
	@echo "---- clean ok ----"

distclean:clean
	@rm -rf *.exe demo $(OBJS_DIR)
    
