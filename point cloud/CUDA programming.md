
https://eximia.co/understanding-the-basics-of-cuda-thread-hierarchies/
The execution configuration allows programmers to specify details about launching the kernel to run in parallel on multiple GPU **threads**. The syntax for this is:

`<<< NUMBER_OF_BLOCKS, NUMBER_OF_THREADS_PER_BLOCK>>>`

**A kernel is executed once for every thread in every thread block configured when the kernel is launched**.

Thus, under the assumption that a kernel called `printHelloGPU` has been defined, the following are true:

- `printHelloGPU<<<1, 1>>>()` is configured to run in a single thread block which has a single thread and will, therefore, run only once.
- `printHelloGPU<<<1, 5>>>()` is configured to run in a single thread block which has 5 threads and will, therefore, run 5 times.
- `printHelloGPU<<<5, 1>>>()` is configured to run in 5 thread blocks which each have a single thread and will, therefore, run five times.
- `printHelloGPU<<<5, 5>>>()` is configured to run in 5 thread blocks which each have five threads and will, therefore, run 25 times.

