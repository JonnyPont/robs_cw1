% for var =5, DF = 1e-10, i =25, numParticles = 25:25:500;


itsResults = [43.5600000000000,33.8400000000000,22.8400000000000,20.1600000000000,17.3600000000000,10.9600000000000,12.7600000000000,8.16000000000000,13.6400000000000,9.20000000000000,9.56000000000000,8.76000000000000,8.04000000000000,8.80000000000000,8.96000000000000,6.84000000000000,9.28000000000000,7.72000000000000,6,10.5600000000000];
convResults = [4.42191385673443,11.8523759196489,3.25955995459990,10.8644865852573,3.22571800111658,6.59626459014902,3.47728177549908,2.73409672757541,2.61785808168650,2.92241328328434,6.33060281556523,2.83876163443581,3.22761002674016,2.57743317828001,2.39332873887724,2.19477127890000,2.54879712138901,2.71057110108185,6.56774215115718,2.40453678070738];
timeResults = [4.66904441809895,5.13556101995739,4.66362288868800,5.20324535873296,5.43383615959927,4.04895111321486,5.44399407335662,3.92343703014339,7.26517379338362,5.45050104059050,6.12371528918643,6.14475261223408,6.07671761980284,7.14380814187769,8.04585448216081,6.57318805787922,9.08399470059183,7.98084636639361,6.53081039883043,12.0573815774455];


%for numParticles = 250, DF = 1e-10, i = 25, var = 5:5:100

itsResults  = [12,7.32000000000000,5.52000000000000,7.32000000000000,7.20000000000000,8.28000000000000,8.32000000000000,10.8800000000000,9.92000000000000,13.0400000000000,12.2400000000000,10.1200000000000,11.0800000000000,12.4800000000000,12.6800000000000,15.6800000000000,17.0800000000000,17.0800000000000,18.8800000000000,19.4400000000000];
convResults = [2.33194077054914,3.91679712750610,3.68308237649263,3.92793250688454,3.16674291288985,3.55735218482087,3.64210931968262,3.28369563428174,3.44124609907622,3.29844494123039,3.42825679856657,2.99870567870118,7.29854110299588,2.40337318358894,3.15775281721709,2.77619336869179,2.47271811414605,2.93291567529878,2.42355654257716,6.67757983874720];
timeResults = [0.885851477266553,0.557037480585794,0.447091185777223,0.552872449167509,0.534427751803028,0.607702521303205,0.610226296982014,0.776297598309091,0.711705237377529,0.924818873069839,0.861023553046777,0.723885503317965,0.787684761794666,0.876501238242531,0.889140757051017,1.09198741299391,1.17783139261168,1.18190150729726,1.36279914652251,1.34447468868894];


%for numParticles = 250,  i = 25, var = 35, DF = 1e-20 (increasing by
%factors of 10. It couldn't complete the iteration with DF = 0.01

itsResults = [8.32000000000000,8.28000000000000,6.68000000000000,8.16000000000000,7.24000000000000,6.92000000000000,9.64000000000000,7.96000000000000,8.68000000000000,8.60000000000000,10.7600000000000,9.20000000000000,9.68000000000000,9.88000000000000,12,9.96000000000000,25.6800000000000];
timeResults = [0.624138665659201,0.604339052721672,0.498912687764866,0.597821189099382,0.539061561312424,0.514611466078427,0.692113166359917,0.587478211781592,0.638733976784274,0.623770172267233,0.764751933944839,0.671173152290886,0.695155461340613,0.709421809478764,0.862158163192744,0.721283323238007,1.74919116196417];
convResults = [3.90142950666001,3.81046833262381,4.40190386181223,4.72543331808225,3.67845221453221,3.59372564915441,3.84873676153684,3.11145261234198,3.77412635587381,3.52742210351662,4.19329185628169,3.70752589772204,3.63331381339141,2.94250837660109,2.95514026082988,2.86509307288226,0.690253570697346];


