function polyTest()

taus=[0.75
    0.5
    1];

p0=[-7.77673e-12
    -1.05638e-13
    1.28438e-13
    -8.06133e-14
    3.96831e-15
    44.0379
    -154.765
    220.551
    -149.424
    40.1398];

p1=[     0.4
    0.683407
    -1.40787
    -1.30785
    6.55326
    -11.4753
    43.8303
    -105.226
    117.954
    -51.5888];

p2=[     0.5
    0.252805
    1.24804
    1.2135
    -5.21775
    9.50847
    -25.9649
    39.5314
    -26.6531
    6.58146];

p0_unC=[ 1.32211e-11
    -1.87927e-14
    -1.92734e-15
    -2.363e-15
    1.56774e-14
    44.0379
    -154.766
    220.551
    -149.425
    40.1398];
p1_unC=[     0.4
    0.683407
    -1.40787
    -1.30785
    6.55326
    -11.4754
    43.8306
    -105.226
    117.954
    -51.5892];
p2_unC=[     0.5
    0.252805
    1.24804
    1.2135
    -5.21776
    9.50857
    -25.9651
    39.5317
    -26.6532
    6.58149];
p0_unC_Sparse=[ 1.32626e-11
    -1.8671e-14
    2.89229e-15
    -8.78493e-16
    -0
    44.0379
    -154.766
    220.551
    -149.425
    40.1398];
p1_unC_Sparse=[     0.4
    0.683407
    -1.40787
    -1.30785
    6.55326
    -11.4754
    43.8306
    -105.226
    117.954
    -51.5892];
p2_unC_Sparse=[     0.5
    0.252805
    1.24804
    1.2135
    -5.21776
    9.50857
    -25.9651
    39.5317
    -26.6532
    6.58149];


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
        
        
        plot(t0,x0,color)
        plot(t1,x1,color)
        plot(t2,x2,color)
 
    end

    figure(1),clf,hold on
    plotPolys(p0,p1,p2,'r');
    figure(2),clf,hold on
    plotPolys(p0_unC,p1_unC,p2_unC,'b');
    figure(3),clf,hold on
    plotPolys(p0_unC_Sparse,p1_unC_Sparse,p2_unC_Sparse,'k');


end

