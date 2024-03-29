package ex04_stream_connector
  extends Modelica.Icons.Package;
  import SI = Modelica.SIunits;
  package StandardWater = Modelica.Media.Water.StandardWater;

  connector Flange
    flow SI.MassFlowRate w "mass flowrate";
    SI.Pressure p "connector pressure";
    stream SI.SpecificEnthalpy h_outflow "specific enthalpy if w<0";
    annotation(
      Icon(graphics = {Rectangle(fillColor = {40, 0, 204}, fillPattern = FillPattern.Solid, extent = {{-100, 100}, {100, -100}})}));
  end Flange;

  model PressureBC
    parameter SI.Pressure p0 = 1e5 "prescribed pressure";
    parameter SI.SpecificEnthalpy hnom = 1e5 "nominal specific enthalpy";
    Flange flange annotation(
      Placement(visible = true, transformation(origin = {98, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    flange.p = p0;
    flange.h_outflow = hnom;
    annotation(
      Icon(graphics = {Ellipse(fillColor = {0, 76, 143}, fillPattern = FillPattern.Sphere, extent = {{-100, 98}, {100, -98}}, endAngle = 360), Text(origin = {-1, 7}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, extent = {{-81, 41}, {81, -41}}, textString = "%name")}, coordinateSystem(initialScale = 0.1)));
  end PressureBC;

  model Tank
    package Medium = StandardWater;
    Medium.ThermodynamicState liquidState(p(start = pstart), h(start = hstart));
    parameter SI.Area A "bottom area";
    parameter SI.Pressure pstart "initial pressure at tank bottom";
    parameter SI.SpecificEnthalpy hstart "initial enthalpy";
    constant SI.Acceleration g = 9.81 "gravity";
    constant SI.Pressure patm = 1 "atmospheric tank";
    SI.Volume V "liquid volume inside tank";
    SI.Mass M "liquid mass inside tank";
    SI.Energy E "Mass total energy";
    SI.SpecificEnthalpy h, ha, hb "fluid specific enthalpies";
    SI.Pressure p "bottom pressure";
    SI.Density d "liquid density";
  
    Flange fl_a annotation(
      Placement(visible = true, transformation(origin = {-98, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-98, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flange fl_b annotation(
      Placement(visible = true, transformation(origin = {98, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealOutput M_meas annotation(
      Placement(visible = true, transformation(origin = {104, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {104, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    liquidState = Medium.setState_ph(p, h);
    d = Medium.density(liquidState);
    M = V * d "liquid m-ass";
    E = M * Medium.specificInternalEnergy(liquidState);
    der(M) = fl_a.w + fl_b.w "mass balance";
    der(E) = fl_a.w * ha + fl_b.w * hb "energy balance";
//I consider the liquid water so
//enthalpy and energy are equivalent
    p = patm + M * g / A "Stevino law";
    fl_a.p = p;
    fl_a.h_outflow = h;
    ha = actualStream(fl_a.h_outflow);
    fl_b.p = p;
    fl_b.h_outflow = h;
    hb = actualStream(fl_b.h_outflow);
    M_meas = M;
  initial equation
    pstart = patm * M * g / A "Stevino law";
    E = hstart * M;
    annotation(
      Icon(graphics = {Rectangle(origin = {0, -1}, fillColor = {4, 120, 152}, fillPattern = FillPattern.VerticalCylinder, extent = {{-100, 99}, {100, -99}})}));
  end Tank;

  model LinValve
    parameter Real Av(unit = "m.s") "flow coefficient";
    Flange fl_a annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Flange fl_b annotation(
      Placement(visible = true, transformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {98, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput theta annotation(
      Placement(visible = true, transformation(origin = {4, 46}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 20}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));
  equation
    fl_a.w + fl_b.w = 0 "mass balance";
    fl_a.w = theta * Av * (fl_a.p - fl_b.p) "valve flow";
    fl_a.h_outflow = inStream(fl_b.h_outflow);
    fl_b.h_outflow = inStream(fl_a.h_outflow);
    annotation(
      Icon(graphics = {Polygon(lineColor = {0, 0, 255}, fillPattern = FillPattern.VerticalCylinder, points = {{-100, 60}, {98, -60}, {100, 60}, {-100, -60}, {-100, 60}}), Text(origin = {-1, -67}, extent = {{-65, 21}, {65, -21}}, textString = "%name")}, coordinateSystem(initialScale = 0.1)));
  end LinValve;

  model Mixer
  extends MixerBase(p_c.p0 = 300000, p_h.hnom = 2e5, p_h.p0 = 300000, tank.A = 0.3 * 0.3, tank.hstart = 1e5, tank.pstart = 150000, vlv_c.Av = 1e-3, vlv_h.Av = 1e-3, vlv_M.Av = 8e-3);
  Modelica.Blocks.Sources.Constant const(k = 1)  annotation(
      Placement(visible = true, transformation(origin = {-84, 36}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Sources.Constant const1(k = 0.5)  annotation(
      Placement(visible = true, transformation(origin = {40, 26}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Ramp ramp(duration = 5, height = 1, startTime = 30)  annotation(
      Placement(visible = true, transformation(origin = {-86, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Feedback feedback annotation(
      Placement(visible = true, transformation(origin = {-14, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(feedback.y, vlv_c.theta) annotation(
      Line(points = {{-4, 70}, {-52, 70}, {-52, 8}, {-52, 8}}, color = {0, 0, 127}));
    connect(feedback.u2, const.y) annotation(
      Line(points = {{-14, 62}, {-14, 42.5}, {-84, 42.5}, {-84, 47}}, color = {0, 0, 127}));
    connect(ramp.y, feedback.u1) annotation(
      Line(points = {{-74, 78}, {-56, 78}, {-56, 70}, {-22, 70}}, color = {0, 0, 127}));
    connect(vlv_h.theta, ramp.y) annotation(
      Line(points = {{-52, -30}, {-76, -30}, {-76, 78}, {-74, 78}}, color = {0, 0, 127}));
    connect(vlv_M.theta, const1.y) annotation(
      Line(points = {{40, -8}, {40, -8}, {40, 14}, {40, 14}}, color = {0, 0, 127}));
  end Mixer;

  partial model MixerBase
    ex04_stream_connector.PressureBC p_c annotation(
      Placement(visible = true, transformation(origin = {-90, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex04_stream_connector.PressureBC p_h annotation(
      Placement(visible = true, transformation(origin = {-92, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex04_stream_connector.LinValve vlv_c annotation(
      Placement(visible = true, transformation(origin = {-52, 6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex04_stream_connector.LinValve vlv_h annotation(
      Placement(visible = true, transformation(origin = {-52, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex04_stream_connector.Tank tank annotation(
      Placement(visible = true, transformation(origin = {0, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex04_stream_connector.LinValve vlv_M annotation(
      Placement(visible = true, transformation(origin = {40, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ex04_stream_connector.PressureBC p_M annotation(
      Placement(visible = true, transformation(origin = {80, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    equation
    connect(p_M.flange, vlv_M.fl_b) annotation(
      Line(points = {{70, -10}, {50, -10}, {50, -10}, {50, -10}}));
    connect(tank.fl_b, vlv_M.fl_a) annotation(
      Line(points = {{10, -12}, {30, -12}, {30, -10}, {30, -10}}));
    connect(vlv_h.fl_b, tank.fl_a) annotation(
      Line(points = {{-42, -32}, {-10, -32}, {-10, -12}, {-10, -12}}));
    connect(vlv_c.fl_b, tank.fl_a) annotation(
      Line(points = {{-42, 6}, {-10, 6}, {-10, -12}, {-10, -12}}));
    connect(p_c.flange, vlv_c.fl_a) annotation(
      Line(points = {{-80, 10}, {-62, 10}, {-62, 6}, {-62, 6}}));
    connect(p_h.flange, vlv_h.fl_a) annotation(
      Line(points = {{-82, -30}, {-62, -30}, {-62, -32}, {-62, -32}}));
    annotation(
      Icon(graphics = {Rectangle(extent = {{-100, 100}, {100, -100}}), Text(origin = {-3, 2}, extent = {{-63, 22}, {63, -22}}, textString = "Mixer")}));
  end MixerBase;
  annotation(
    uses(Modelica(version = "3.2.3")));
end ex04_stream_connector;
