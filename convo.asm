# Initialize registers
00000293    # addi x5, x0, 0    # i (input index)
00000313    # addi x6, x0, 0    # j (kernel index)
00000393    # addi x7, x0, 0    # k (output index)

# outer_loop:
00000313    # addi x6, x0, 0    # Reset kernel index

# inner_loop:
00628e33    # add x28, x5, x6   # x28 = i + j

01ce1e33    # sll x28, x28, x28 # x28 = (i + j) * 4 (assuming x28 is initially 2)
00ae0e33    # add x28, x28, x10 # x28 = &input[i + j]
000e2f03    # lw x30, 0(x28)    # x30 = input[i + j]

00631e33    # sll x28, x6, x6   # x28 = j * 4 (assuming x6 is initially 2)
00be0e33    # add x28, x28, x11 # x28 = &kernel[j]
000e2f83    # lw x31, 0(x28)    # x31 = kernel[j]

00000393    # addi x7, x0, 0    # x7 = 0 (accumulator for multiplication)
020f0c63    # beq x30, x0, skip_mult #0000001 00000 11110 000 11000 1100011 imm=000000011100
020f8a63    # beq x31, x0, skip_mult

# mult_loop:
01f38393    # addi x7, x7, x30 #wrong
ffff8f93    # addi x31, x31, -1
fe0f94e3    # bne x31, x0, mult_loop #1111111 00000 11111 001 01001 1100011 imm=111111110100

# skip_mult:
007e0e33    # add x28, x28, x7  # x28 = &output[k]
000e2f03    # lw x30, 0(x28)    # x30 = output[k]
007f0f33    # add x30, x30, x7  # x30 += result of multiplication
01cf2023    # sw x30, 0(x28)    # Store result back to output[k]

00130313    # addi x6, x6, 1
fc734ce3    # blt x6, x13, inner_loop

00128293    # addi x5, x5, 1
00438393    # addi x7, x7, 4

40c50e33    # sub x28, x10, x12
001e0e13    # addi x28, x28, 1  # x28 = input_length - kernel_length + 1
fc5346e3    # blt x5, x28, outer_loop

# End of convolution
00000073    # ecall