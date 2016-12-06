#!env python

import scipy.stats
vals = list(map(lambda a: scipy.stats.chi2.ppf(0.95, a), range(1, 1000)))
print('[NAN, {}]'.format(', '.join(map(str, vals))))
