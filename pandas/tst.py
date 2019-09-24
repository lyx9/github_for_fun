#/usr/bin/python

import pandas as pd
import re
import numpy as np
import matplotlib.pyplot as plt

plt.style.use('ggplot')
#matplotlib inline

pd.set_option("display.max_columns", 30)
pd.set_option("display.max_colwidth", 100)
pd.set_option("display.precision", 3)

csv_path = r"/Users/liyixiang/Downloads/bj.lianjia.com-(Crawl-Run)---2018-11-20T014129Z.csv"

df = pd.read_csv(csv_path)
df.columns
