import matplotlib.pyplot as plt
import random

# Dữ liệu
x = [1, 2, 3, 4, 5]
y1 = [2, 4, 6, 8, 10]
y2 = [1, 3, 5, 7, 9]
y3 = [0, 2, 4, 6, 8]

# Mảng chứa các mã màu sắc ngẫu nhiên
colors = [f"#{random.randint(0, 0xFFFFFF):06x}" for _ in range(3)]

# Vẽ biểu đồ đường với mã màu ngẫu nhiên
plt.plot(x, y1, color=colors[0], label='Line 1')
plt.plot(x, y2, color=colors[1], label='Line 2')
plt.plot(x, y3, color=colors[2], label='Line 3')

# Đặt tên cho trục x và trục y
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

# Đặt tiêu đề cho biểu đồ
plt.title('Multiple Line Chart')

# Hiển thị chú thích
plt.legend()

# Hiển thị biểu đồ
plt.show()
