MODULE Right_ASL_Demo
LOCAL CONST string YuMi_App_Program_Version:="1.0.1"; !Do not edit or remove this line!
LOCAL VAR robtarget s60 := [[537.58,-221.99,107.92],[0.0203865,-0.290011,-0.954645,-0.0642686],[0,-2,0,11],[-176.329,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s59 := [[487.37,-152.04,187.31],[0.0570472,-0.210759,0.974549,-0.0508043],[1,-2,0,11],[-141.076,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s58 := [[358.37,-68.33,133.00],[0.275214,0.0426853,0.863133,0.421233],[0,-2,-1,11],[-117.324,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s57 := [[358.37,-68.33,133.00],[0.275214,0.0426862,0.863132,0.421234],[0,-2,-1,11],[-117.324,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s56 := [[467.55,-213.81,57.09],[0.496621,-0.80295,-0.0521253,-0.325456],[0,0,0,11],[165.649,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s55 := [[306.20,-52.91,16.67],[0.717218,-0.691447,0.0462227,-0.073234],[1,0,0,11],[153.702,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s54 := [[310.32,9.22,5.39],[0.716537,-0.693105,0.0533556,-0.0577434],[1,0,0,11],[153.592,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s53 := [[309.71,9.64,7.16],[0.719408,-0.69009,0.0433761,-0.0659264],[1,0,0,11],[153.586,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s52 := [[306.40,0.86,31.28],[0.734937,-0.673984,0.0222505,-0.0715334],[1,0,0,11],[152.628,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s51 := [[311.66,-63.00,44.82],[0.56004,-0.794065,-0.0460185,-0.231728],[0,0,0,11],[154.727,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s50 := [[299.07,-103.69,-6.20],[0.280816,-0.867914,-0.384884,-0.140468],[0,0,-1,11],[154.742,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s49 := [[352.75,-152.26,-47.58],[0.00513543,0.702291,0.711785,0.0111369],[1,0,-1,1010],[149.493,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s48 := [[345.22,-148.03,-90.26],[0.00526599,-0.716656,-0.697243,-0.0151133],[1,0,-1,1010],[153.525,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s47 := [[345.92,-149.32,-76.57],[0.0108524,-0.753223,-0.657389,-0.0194125],[1,0,-1,1010],[156.115,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s46 := [[357.66,-150.93,-35.49],[0.00970556,0.737723,0.663169,0.126002],[0,0,0,1010],[163.101,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s45 := [[388.73,-144.04,35.15],[0.0733127,-0.627707,-0.616071,-0.470176],[0,-1,0,11],[-163.98,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s44 := [[342.30,-148.50,49.05],[0.350551,-0.529329,-0.492073,-0.595641],[0,-1,0,11],[-159.838,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s43 := [[285.33,-50.36,-15.16],[0.467312,-0.523371,-0.473345,-0.532585],[0,-1,0,11],[-169.176,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s42 := [[284.50,11.38,-19.53],[0.493118,-0.506889,-0.461514,-0.535634],[1,-1,0,11],[-170.503,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s41 := [[284.50,11.39,-19.53],[0.49312,-0.506885,-0.461513,-0.535636],[1,-1,0,11],[-170.503,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s40 := [[283.93,11.08,-19.46],[0.493586,-0.506785,-0.461377,-0.535419],[1,-1,0,11],[-170.522,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s39 := [[283.93,11.07,-19.46],[0.493582,-0.506788,-0.461378,-0.535418],[1,-1,0,11],[-170.522,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s38 := [[283.93,11.07,-19.47],[0.493573,-0.5068,-0.461384,-0.53541],[1,-1,0,11],[-170.522,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s37 := [[282.43,12.65,-10.40],[0.493036,-0.507364,-0.460348,-0.536261],[1,-1,0,11],[-169.512,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s36 := [[285.29,11.12,13.96],[0.483333,-0.506561,-0.479515,-0.529009],[0,-1,0,11],[-167.842,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s35 := [[299.20,-72.45,52.01],[0.505778,-0.516645,-0.4521,-0.522371],[0,-1,0,11],[-163.463,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s34 := [[301.58,-60.87,67.73],[0.531821,-0.485612,-0.484682,-0.496418],[0,-1,0,11],[-162.383,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s33 := [[315.49,-68.53,58.63],[0.496532,-0.515442,-0.489328,-0.49833],[0,-1,0,11],[-161.717,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s32 := [[315.48,-68.53,58.63],[0.496531,-0.515444,-0.489327,-0.49833],[0,-1,0,11],[-161.717,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s31 := [[335.43,-129.87,89.30],[0.417557,-0.554213,-0.666307,-0.273002],[0,-1,0,11],[-161.728,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s30 := [[345.66,-189.94,55.79],[0.0163252,-0.262519,-0.955378,-0.134427],[0,-2,0,11],[-147.912,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget p1 := [[356.51,-126.17,141.37],[0.423246,-0.466449,-0.566306,-0.531588],[0,-1,0,11],[-140.156,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s29 := [[449.35,-192.96,196.47],[0.327645,0.144187,0.899395,0.250893],[0,-2,0,11],[-149.178,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s28 := [[278.96,-52.84,69.35],[0.598615,-0.575848,-0.497068,-0.250963],[0,0,0,11],[150.45,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s27 := [[285.53,-146.72,-90.22],[0.000644566,7.67705E-05,-0.999188,0.0402885],[0,-2,0,11],[-155.028,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s26 := [[550.62,-141.67,99.35],[0.652704,-0.574035,0.494275,-0.0124307],[1,-3,0,1010],[-151.612,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s25 := [[278.92,-94.43,7.55],[0.697371,-0.707889,0.093585,-0.0617177],[0,-3,-1,1010],[-141.306,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s24 := [[305.50,6.82,0.13],[0.706805,-0.703188,0.0586106,-0.0501884],[1,-3,-1,1010],[-144.867,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s23 := [[296.14,-66.88,48.95],[0.72547,-0.683068,0.0742921,-0.0398991],[0,-3,-1,1010],[-139.643,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s22 := [[286.15,-99.41,71.37],[0.441217,-0.740625,0.337914,0.377645],[1,-2,-1,1010],[141.655,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s21 := [[342.14,-167.41,-21.04],[0.0100919,0.732467,-0.680595,0.013438],[1,-2,0,11],[129.399,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s20 := [[341.36,-149.04,-88.24],[0.0157489,-0.697202,0.716673,-0.00638766],[1,-2,0,11],[137.829,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s19 := [[349.88,-145.28,-65.05],[0.013809,-0.688714,0.724545,-0.0227401],[1,-2,0,11],[136.089,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s18 := [[330.54,-213.14,14.27],[0.0111853,-0.783983,0.616636,0.0707475],[1,-2,0,11],[120.453,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s17 := [[293.17,-119.78,-8.05],[0.278234,-0.65326,-0.636422,-0.301337],[1,-4,-1,11],[128.84,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s16 := [[287.02,-24.16,-12.90],[0.420715,-0.570407,-0.41323,-0.57173],[1,-4,-1,11],[134.835,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s15 := [[287.98,20.08,-17.93],[0.439184,-0.556202,-0.441673,-0.550166],[1,-4,-1,11],[132.875,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s14 := [[287.99,20.09,-17.93],[0.439187,-0.556199,-0.441664,-0.550172],[1,-4,-1,11],[132.875,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s13 := [[287.99,20.09,-17.93],[0.43919,-0.556198,-0.441664,-0.550172],[1,-4,-1,11],[132.875,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s12 := [[287.98,20.09,-17.93],[0.439193,-0.556204,-0.441676,-0.550154],[1,-4,-1,11],[132.875,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s11 := [[291.53,18.15,3.43],[0.454261,-0.541273,-0.401125,-0.582897],[1,-4,-1,11],[133.435,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s10 := [[344.84,-46.36,58.46],[0.437354,-0.537831,-0.404893,-0.596256],[1,-4,-1,11],[129.912,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s9 := [[344.79,-46.36,58.29],[0.437236,-0.538047,-0.40519,-0.595945],[1,-4,-1,11],[129.908,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s8 := [[387.55,-123.27,88.22],[0.382696,-0.608278,-0.391489,-0.574698],[1,-4,-1,11],[132.886,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s7 := [[377.41,-175.66,57.46],[0.371254,0.382215,-0.8238,0.193485],[1,-2,-1,11],[-160.37,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s6 := [[253.69,-227.69,32.72],[0.0393185,-0.0503203,0.995728,-0.0666949],[0,-2,0,11],[-149.635,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s5 := [[285.55,-146.72,-90.22],[0.000628571,4.59053E-05,-0.999188,0.0402877],[0,-2,0,11],[-155.026,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s4 := [[285.71,-166.06,-66.31],[0.0164553,0.0116342,0.999003,-0.0398251],[0,-2,0,11],[-153.319,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s3 := [[285.56,-146.75,-90.22],[0.00061882,2.02596E-05,-0.999187,0.0403124],[0,-2,0,11],[-155.025,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s2 := [[283.94,-148.10,-80.96],[0.00559929,-0.00185021,0.999284,-0.0373822],[0,-2,0,11],[-154.675,9E+09,9E+09,9E+09,9E+09,9E+09]];
LOCAL VAR robtarget s1 := [[303.87,-127.03,37.37],[0.0164974,0.00923537,0.992408,0.12153],[0,-2,0,11],[-150.299,9E+09,9E+09,9E+09,9E+09,9E+09]];
PROC main()
MoveSync s57;
OpenHand;
MoveSync s1;
MoveSync s2;
MoveSync s3;
MoveSync s5;
MoveSync s27;
CloseHand;
MoveSync s6;
MoveSync s30;
MoveSync s31;
MoveSync s32;
MoveSync s33;
MoveSync s34;
MoveSync s35;
MoveSync s36;
MoveSync s37;
MoveSync s38;
MoveSync s39;
MoveSync s40;
MoveSync s41;
MoveSync s42;
OpenHand;
MoveSync s43;
MoveSync s44;
MoveSync s45;
MoveSync s46;
MoveSync s47;
MoveSync s48;
CloseHand;
MoveSync s49;
MoveSync s50;
MoveSync s51;
MoveSync s52;
MoveSync s54;
OpenHand;
MoveSync s55;
MoveSync s56;
MoveSync s60;
MoveSync s58;
ENDPROC
ENDMODULE