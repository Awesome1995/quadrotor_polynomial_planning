function polyTest()
taus=[0.75
0.5
1];
p0=[-9.70815e-11
6.16429e-13
2.5
-1.30704e-12
1.26986e-13
557.254
-2268.35
3692.18
-2810.26
833.308];
p1=[4
5.33602
-13.0196
-4.21641
141.248
-892.632
3533.35
-7596.88
8309.3
-3653.38];
p2=[5
3.31741
11.6081
8.11774
-108.608
412.772
-899.421
1029.29
-579.862
127.789];
p0_unC=[2.27011e-10
-7.98943e-14
2.5
-3.10605e-14
-0
557.254
-2268.34
3692.17
-2810.25
833.306];
p1_unC=[4
5.33602
-13.0196
-4.2164
141.248
-892.627
3533.33
-7596.85
8309.28
-3653.37];
p2_unC=[5
3.31741
11.6081
8.11775
-108.608
412.773
-899.422
1029.29
-579.863
127.789];
p0_unC_sparse=[2.27557e-10
-3.88652e-14
2.5
-2.14075e-14
1.25419e-13
557.254
-2268.34
3692.17
-2810.25
833.306];
p1_unC_sparse=[4
5.33602
-13.0196
-4.2164
141.248
-892.627
3533.33
-7596.85
8309.28
-3653.37];
p2_unC_sparse=[5
3.31741
11.6081
8.11775
-108.608
412.773
-899.422
1029.29
-579.863
127.789];




    function [t0, x0, t1, x1, t2, x2] = genPlotPoints(p_0, p_1, p_2)
        
        t0_eval = linspace(0,taus(1));
        x0 = polyval(flipud(p_0),t0_eval);
        t0 = t0_eval;
        
        t1_eval = linspace(0,taus(2));
        x1 = polyval(flipud(p_1),t1_eval);
        t1 = t1_eval + taus(1);
        
        t2_eval = linspace(0,taus(3));
        x2 = polyval(flipud(p_2),t2_eval);
        t2 = t2_eval + taus(1) + taus(2);
        
    end


    function plotPolys(p_0, p_1, p_2, color)
        [t0, x0, t1, x1, t2, x2] = genPlotPoints(p_0,p_1,p_2);
        
        plot(t0,x0,'k')
        plot(t1,x1,'r')
        plot(t2,x2,'b')
 
    end

    figure(1),clf,hold on
    plotPolys(p0,p1,p2,'r');
    figure(2),clf,hold on
    plotPolys(p0_unC,p1_unC,p2_unC,'b');
    figure(3),clf,hold on
    plotPolys(p0_unC_sparse,p1_unC_sparse,p2_unC_sparse,'k');


end

