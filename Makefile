#!make -f

CXX=g++
CXXFLAGS=-std=c++11 -Werror -Wsign-conversion
VALGRIND_FLAGS=-v --leak-check=full --show-leak-kinds=all  --error-exitcode=99

COMMON_SOURCES=Graph.cpp Algorithms.cpp
DEMO_SOURCES=Demo.cpp $(COMMON_SOURCES)
TEST_SOURCES=TestCounter.cpp Test.cpp $(COMMON_SOURCES)

DEMO_OBJECTS=$(subst .cpp,.o,$(DEMO_SOURCES))
TEST_OBJECTS=$(subst .cpp,.o,$(TEST_SOURCES))

run: demo
	./$^

demo: $(DEMO_OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o demo

test: $(TEST_OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o test

tidy:
	clang-tidy $(DEMO_SOURCES) $(TEST_SOURCES) -checks=bugprone-*,clang-analyzer-*,cppcoreguidelines-*,performance-*,portability-*,readability-*,-cppcoreguidelines-pro-bounds-pointer-arithmetic,-cppcoreguidelines-owning-memory,-misc-no-recursion,-bugprone-easily-swappable-parameters,-readability-identifier-length,-cppcoreguidelines-avoid-magic-numbers --warnings-as-errors=-* --
valgrind: demo test
	valgrind --tool=memcheck $(VALGRIND_FLAGS) ./demo 2>&1 | { egrep "lost| at " || true; }
	valgrind --tool=memcheck $(VALGRIND_FLAGS) ./test 2>&1 | { egrep "lost| at " || true; }

%.o: %.cpp
	$(CXX) $(CXXFLAGS) --compile $< -o $@

clean:
	rm -f *.o demo test