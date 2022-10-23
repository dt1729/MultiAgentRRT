import csv
import matplotlib.pyplot as plt


def time_plot(names):
    data = [[] for _ in range(csv_col)]
    count = 0
    for name in names:
        with open(name) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                data[count].append(float(row[0])/10**6)
        print(count, data[count])
        count += 1
    
    # Creating axes instance
    fig1, ax1 = plt.subplots()
    ax1.set_title('Run Time Variation')
    ax1.boxplot(data)
    ax1.set_xlabel("Number of Agents")
    ax1.set_ylabel("Time taken to run")
    # show plot
    plt.show()

    means = []

    for d in data:
        means.append(sum(d)/len(d))
    
    xs = [i for i in range(1,6)]
    plt.scatter(xs, means)
    plt.xlabel("Number of agents")
    plt.ylabel("Mean time to run")
    plt.title("Mean time taken with increasing agent count")
    plt.show()

csv_col = 5
times = ["time_rrt_2.csv","time_rrt_3.csv","time_rrt_4.csv","time_rrt_5.csv","time_rrt_6.csv"]
valid = ["valid_rrt_2.csv","valid_rrt_3.csv","valid_rrt_4.csv","valid_rrt_5.csv","valid_rrt_6.csv"]
sizes = ["rrt_size_2.csv","rrt_size_3.csv","rrt_size_4.csv","rrt_size_5.csv","rrt_size_6.csv"]
time_plot(times)
