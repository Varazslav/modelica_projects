package ex05_controller
  extends Modelica.Icons.Package;
  import SI = Modelica.SIunits;

  model DigitalPI
    parameter Boolean useTrack = false "use input connection for track" annotation(
      choices(checkBox = true));
    parameter SI.Time Ts "sampling time";
    // PI parameters
    parameter SI.PerUnit kp "proportional gain";
    parameter SI.Time Ti "integral time";
    parameter Real PVmax = 1 "max PV value";
    parameter Real PVmin = 0 "min PV value";
    parameter Real COmax = 1 "max CO value";
    parameter Real COmin = 0 "min CO value";
    parameter Real I_start = 0 "integrator initial state";
    final parameter Real spanPV = PVmax - PVmin;
    final parameter Real spanCO = COmax - COmin;
    // continuous variables
    SI.PerUnit SPn, PVn "normalized Sp and PV";
    SI.PerUnit TVn "normalized track value";
    SI.PerUnit errn "normalized error";
    // digital variables
    discrete SI.PerUnit errnd "digital normalized error";
    discrete SI.PerUnit Pnd "proportional control";
    discrete SI.PerUnit Ind "integral control";
    discrete SI.PerUnit CObsnd "CO before saturation";
    discrete SI.PerUnit COasnd "CO after saturation";
    discrete SI.PerUnit TVnd "track value";
    Modelica.Blocks.Interfaces.RealInput SP annotation(
      Placement(visible = true, transformation(origin = {-106, 34}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-106, 34}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput PV annotation(
      Placement(visible = true, transformation(origin = {-104, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-104, -64}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput CO annotation(
      Placement(visible = true, transformation(origin = {106, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {106, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.BooleanInput TS if useTrack annotation(
      Placement(visible = true, transformation(origin = {-220, 36}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-40, 102}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
    Modelica.Blocks.Interfaces.RealInput TV if useTrack annotation(
      Placement(visible = true, transformation(origin = {-72, 112}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {50, 102}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  protected
    Modelica.Blocks.Interfaces.BooleanInput TS_internal;
    Modelica.Blocks.Interfaces.RealInput TV_internal;
  algorithm
    when sample(0, Ts) then
      errnd := errn;
      TVnd := TVn;
      Pnd := kp * errnd;
      if pre(TS_internal) then
        CObsnd := TVnd;
        Ind := TVnd;
      else
        Ind := pre(Ind) + kp / Ti * errnd * Ts;
        CObsnd := Pnd + Ind;
      end if;
      COasnd := min(1, max(0, CObsnd));
      if COasnd < CObsnd or COasnd > CObsnd then
        Ind := pre(Ind);
      end if;
    end when;
// A/D converter
// PI
// track is active
// if the saturation is active I stop the integrator
// basic anti windup configuration
  initial algorithm
    Ind := (I_start - COmin) / spanCO;
  equation
// normalization
    SPn = (SP - PVmin) / spanPV;
    PVn = (PV - PVmin) / spanPV;
    TVn = (TV_internal - COmin) / spanCO;
// I/O calculations
    errn = SPn - PVn;
    CO = COasnd * spanCO + COmin "D/A conversion";
    connect(TS, TS_internal);
// TS and TV are instantiated only if useTrack is true,
    connect(TV, TV_internal);
// so if it is false this connection does not occur
    if not useTrack then
      TS_internal = false;
      TV_internal = 0;
    end if;
    annotation(
      Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-15, 25}, extent = {{-57, 39}, {85, -73}}, textString = "PI\nDIGITAL"), Text(origin = {-18, -121}, extent = {{-42, 15}, {74, -23}}, textString = "%name")}));
  end DigitalPI;

  model SimpleTank
    parameter SI.Area A = 1 "tank section";
    parameter SI.Density d = 1000 "liquid density";
    parameter SI.Length h_start = 5 "start level";
    parameter Real Av = 10 "flow coefficient of output valve";
    
    SI.Length h "level";
    SI.Mass M(start = d*A*h_start);
    SI.MassFlowRate wo "output flowrate";
  
    Modelica.Blocks.Interfaces.RealInput wi annotation(
      Placement(visible = true, transformation(origin = {-100, 84}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-100, 84}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput level annotation(
      Placement(visible = true, transformation(origin = {106, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-110, 18}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Interfaces.RealInput theta annotation(
      Placement(visible = true, transformation(origin = {142, -48}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {140, -70}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  
  equation
    der(M) = wi - wo;
    wo = theta*Av*sqrt(d*9.81*h); // Stevino law
    M = d*A*h;
    
    level = h;

  annotation(
      Icon(graphics = {Rectangle(fillColor = {0, 190, 207}, fillPattern = FillPattern.VerticalCylinder, extent = {{-100, 100}, {100, -100}}), Rectangle(origin = {138, -91}, extent = {{-38, 9}, {38, -9}}), Text(origin = {-51, -102}, extent = {{-31, 6}, {119, -30}}, textString = "%name")}, coordinateSystem(initialScale = 0.1)));
  end SimpleTank;

  model ControlledTank
  ex05_controller.SimpleTank simpleTank1 annotation(
      Placement(visible = true, transformation(origin = {-56, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex05_controller.SimpleTank simpleTank2 annotation(
      Placement(visible = true, transformation(origin = {12, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex05_controller.DigitalPI digitalPI1(PVmax = 100, Ti = 20, Ts = 1, kp = -4)  annotation(
      Placement(visible = true, transformation(origin = {-56, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant LevelSP(k = 5)  annotation(
      Placement(visible = true, transformation(origin = {-94, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Step wiStep(height = 1000, startTime = 10)  annotation(
      Placement(visible = true, transformation(origin = {-100, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.PI contPI(T = 20, k = -4)  annotation(
      Placement(visible = true, transformation(origin = { 56, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback1 annotation(
      Placement(visible = true, transformation(origin = {-18, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain norm(k = 1 / 100)  annotation(
      Placement(visible = true, transformation(origin = {20, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(wiStep.y, simpleTank2.wi) annotation(
      Line(points = {{-88, 2}, {-82, 2}, {-82, -30}, {-10, -30}, {-10, 2}, {2, 2}, {2, 2}}, color = {0, 0, 127}));
  connect(contPI.y, simpleTank2.theta) annotation(
      Line(points = {{68, 54}, {82, 54}, {82, 18}, {26, 18}, {26, -13}}, color = {0, 0, 127}));
  connect(simpleTank2.level, feedback1.u2) annotation(
      Line(points = {{1, -4}, {-18, -4}, {-18, 46}}, color = {0, 0, 127}));
    connect(norm.y, contPI.u) annotation(
      Line(points = {{32, 54}, {44, 54}, {44, 54}, {44, 54}}, color = {0, 0, 127}));
    connect(feedback1.y, norm.u) annotation(
      Line(points = {{-8, 54}, {8, 54}}, color = {0, 0, 127}));
  connect(LevelSP.y, feedback1.u1) annotation(
      Line(points = {{-82, 54}, {-78, 54}, {-78, 82}, {-34, 82}, {-34, 54}, {-26, 54}}, color = {0, 0, 127}));
  connect(simpleTank1.level, digitalPI1.PV) annotation(
      Line(points = {{-68, -4}, {-78, -4}, {-78, 48}, {-66, 48}}, color = {0, 0, 127}));
  connect(LevelSP.y, digitalPI1.SP) annotation(
      Line(points = {{-82, 54}, {-66, 54}, {-66, 57}, {-67, 57}}, color = {0, 0, 127}));
  connect(digitalPI1.CO, simpleTank1.theta) annotation(
      Line(points = {{-45, 58}, {-42, 58}, {-42, -14}}, color = {0, 0, 127}));
    connect(wiStep.y, simpleTank1.wi) annotation(
      Line(points = {{-89, 2}, {-66, 2}}, color = {0, 0, 127}));
  end ControlledTank;

  model ControlledTankTrk
  extends ControlledTank(digitalPI1.useTrack = true);
  Modelica.Blocks.Sources.BooleanStep trkStep(startTime = 20, startValue = true)  annotation(
      Placement(visible = true, transformation(origin = {-92, 104}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant Tv(k = 1)  annotation(
      Placement(visible = true, transformation(origin = {-18, 104}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  equation
    connect(Tv.y, digitalPI1.TV) annotation(
      Line(points = {{-29, 104}, {-52, 104}, {-52, 64}, {-50, 64}}, color = {0, 0, 127}));
    connect(trkStep.y, digitalPI1.TS) annotation(
      Line(points = {{-80, 104}, {-60, 104}, {-60, 64}, {-60, 64}}, color = {255, 0, 255}));
  end ControlledTankTrk;
  annotation(
    uses(Modelica(version = "3.2.3")));
end ex05_controller;
