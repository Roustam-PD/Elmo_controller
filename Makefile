# Определяем переменные
CXX = g++
CXXFLAGS = -lrt -lpthread -lstdc++
SRC_DIR = ./demo
OUT_DIR = ./

# Исходные файлы
MOTOR_INTERFACE_SRC = motor_interface.cpp
DEMO_EXAMPLE_SRC = $(SRC_DIR)/demo_example.cpp

# Выходные файлы
MOTOR_INTERFACE_OUT = motor_interface
DEMO_EXAMPLE_OUT = $(OUT_DIR)/demo_example

# Правила компиляции
all: $(MOTOR_INTERFACE_OUT) $(DEMO_EXAMPLE_OUT)

$(MOTOR_INTERFACE_OUT): $(MOTOR_INTERFACE_SRC)
	$(CXX) $(MOTOR_INTERFACE_SRC) $(CXXFLAGS) -o $(MOTOR_INTERFACE_OUT)

$(DEMO_EXAMPLE_OUT): $(DEMO_EXAMPLE_SRC)
	$(CXX) $(DEMO_EXAMPLE_SRC) $(CXXFLAGS) -o $(DEMO_EXAMPLE_OUT)

# Очистка выходных файлов
clean:
	rm -f $(MOTOR_INTERFACE_OUT) $(DEMO_EXAMPLE_OUT)

.PHONY: all clean
