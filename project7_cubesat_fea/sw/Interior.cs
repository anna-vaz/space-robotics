// Interior of the 3U CubeSat, driven by cad/interior_spec.txt (written by interior_spec.py).
// Phase "parts": countersinks on the end plate's M3 holes, end_plate_top (copy + 42 mm camera
//               aperture), and every interior part (plates with holes, tubes, mass dummies).
// Phase "asm":   CubeSat_3U.SLDASM = structure + interior, placed by exact transforms and fixed;
//               checks part masses, placement boxes, total mass / CoM / inertia, interferences.
using System;
using System.IO;
using System.Linq;
using System.Globalization;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

public static class Interior
{
    const string Dir = @"D:\cad free projects\CubeSat 3U\";
    static readonly CultureInfo CI = CultureInfo.InvariantCulture;
    static double D(string s) { return double.Parse(s, CI); }

    public static void Run(string outDir)
    {
        string phase = System.Environment.GetEnvironmentVariable("INTERIOR_PHASE") ?? "parts";
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        int tog = (int)swUserPreferenceToggle_e.swInputDimValOnCreate;
        bool old = sw.GetUserPreferenceToggle(tog);
        sw.SetUserPreferenceToggle(tog, false);
        try
        {
            var spec = File.ReadAllLines(Path.Combine(outDir, "..", "cad", "interior_spec.txt"));
            if (phase == "parts") Parts(sw, spec); else Asm(sw, spec);
        }
        finally { sw.SetUserPreferenceToggle(tog, old); }
    }

    // ------------------------------------------------------------------ parts
    static void Parts(SldWorks sw, string[] spec)
    {
        EndPlates(sw);
        foreach (var line in spec.Where(l => l.StartsWith("PART|")))
        {
            var f = line.Split('|');
            string path = Dir + f[1];
            var open = (ModelDoc2)sw.GetOpenDocumentByName(path);
            if (open != null) sw.CloseDoc(open.GetTitle());
            if (File.Exists(path)) File.Delete(path);        // generated parts are rebuilt from the spec
            var doc = NewPart(sw);
            string kind = f[2];
            string mat, lib; double target;
            if (kind == "PLATE")
            {
                double a = D(f[3]) / 1000, b = D(f[4]) / 1000, t = D(f[5]) / 1000;
                Sketch(doc, "Front Plane", sm => { sm.CreateCornerRectangle(0, 0, 0, a, b, 0); });
                Extrude(doc, t, "Body");
                if (f[6] != "-")
                {
                    var holes = f[6].Split(';').Select(h => h.Split(':').Select(D).ToArray()).ToArray();
                    Sketch(doc, "Front Plane", sm => {
                        foreach (var h in holes) sm.CreateCircleByRadius(h[0] / 1000, h[1] / 1000, 0, h[2] / 1000);
                    });
                    CutThrough(doc, "Holes");
                }
                if (f[7] != "-")
                {
                    var bo = f[7].Split(':').Select(D).ToArray();
                    doc.ClearSelection2(true);
                    doc.Extension.SelectByID2("Front Plane", "PLANE", 0, 0, 0, false, 0, null, 0);
                    var pl = (Feature)doc.FeatureManager.InsertRefPlane(8, t, 0, 0, 0, 0);
                    pl.Name = "Top face plane";
                    Sketch(doc, pl.Name, sm => { sm.CreateCircleByRadius(bo[0] / 1000, bo[1] / 1000, 0, bo[2] / 1000); });
                    Extrude(doc, bo[3] / 1000, "Boss");
                }
                mat = f[8]; lib = f[9]; target = D(f[10]);
            }
            else
            {
                double ro = D(f[3]) / 1000, ri = D(f[4]) / 1000, L = D(f[5]) / 1000;
                Sketch(doc, "Front Plane", sm => {
                    sm.CreateCircleByRadius(0, 0, 0, ro);
                    if (ri > 0) sm.CreateCircleByRadius(0, 0, 0, ri);
                });
                Extrude(doc, L, "Body");
                mat = f[6]; lib = f[7]; target = D(f[8]);
            }
            ((PartDoc)doc).SetMaterialPropertyName2("", lib, mat);
            string db; string got = ((PartDoc)doc).GetMaterialPropertyName2("", out db);
            doc.EditRebuild3();
            double m = ((MassProperty)doc.Extension.CreateMassProperty()).Mass * 1e3;
            int e = 0, w = 0;
            doc.Extension.SaveAs(path, 0, 1, null, ref e, ref w);
            Console.WriteLine(string.Format("{0,-24} {1,9:F3} g (target {2,9:F3})  material '{3}'  {4}",
                f[1], m, target, got, Math.Abs(m - target) < 0.01 ? "OK" : "MISMATCH"));
            sw.CloseDoc(doc.GetTitle());
        }
    }

    static void EndPlates(SldWorks sw)
    {
        int e = 0, w = 0;
        string bot = Dir + "end_plate.SLDPRT", top = Dir + "end_plate_top.SLDPRT";
        var doc = (ModelDoc2)sw.GetOpenDocumentByName(bot) ?? (ModelDoc2)sw.OpenDoc6(bot, 1, 0, "", ref e, ref w);
        if (doc.GetSaveFlag()) throw new Exception("end_plate has unsaved changes - nothing done");
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        double v0 = ((MassProperty)doc.Extension.CreateMassProperty()).Volume;
        if (((PartDoc)doc).FeatureByName("M3 countersinks") == null)
        {
            doc.ClearSelection2(true);
            int n = 0;
            foreach (Body2 b in (object[])((PartDoc)doc).GetBodies2(0, true))
                foreach (Face2 fc in (object[])b.GetFaces())
                {
                    double[] bx = (double[])fc.GetBox();
                    if (!((Surface)fc.GetSurface()).IsPlane() || Math.Abs(bx[2]) > 1e-9 || Math.Abs(bx[5]) > 1e-9) continue;
                    foreach (Edge ed in (object[])fc.GetEdges())
                    {
                        var c = (Curve)ed.GetCurve();
                        if (c.IsCircle() && Math.Abs(((double[])c.CircleParams)[6] - 0.0017) < 1e-7)
                        { ((Entity)ed).Select4(true, null); n++; }
                    }
                }
            var ch = (Feature)doc.FeatureManager.InsertFeatureChamfer(4, 1, 0.00166, Math.PI / 4, 0, 0, 0, 0);
            if (ch == null) throw new Exception("countersink chamfer failed");
            ch.Name = "M3 countersinks";
            double v1 = ((MassProperty)doc.Extension.CreateMassProperty()).Volume;
            Console.WriteLine(string.Format("end_plate: countersunk {0} M3 holes (90 deg, dia 6.72) on the outer face, volume -{1:F1} mm^3", n, (v0 - v1) * 1e9));
            doc.Save3(1, ref e, ref w);
        }
        else Console.WriteLine("end_plate: countersinks already present");
        if (!File.Exists(top))
        {
            doc.Extension.SaveAs(top, 0, 2 | 1, null, ref e, ref w);           // copy, silent
            var t = (ModelDoc2)sw.OpenDoc6(top, 1, 0, "", ref e, ref w);
            sw.ActivateDoc3(t.GetTitle(), false, 0, ref e);
            Sketch(t, "Front Plane", sm => { sm.CreateCircleByRadius(0.0485, 0.0475, 0, 0.021); });
            CutThrough(t, "Camera aperture");
            t.Save3(1, ref e, ref w);
            Console.WriteLine(string.Format("end_plate_top: aperture dia 42, mass {0:F3} g",
                ((MassProperty)t.Extension.CreateMassProperty()).Mass * 1e3));
            sw.CloseDoc(t.GetTitle());
        }
        Console.WriteLine(string.Format("end_plate mass {0:F3} g", ((MassProperty)doc.Extension.CreateMassProperty()).Mass * 1e3));
    }

    static ModelDoc2 NewPart(SldWorks sw)
    {
        var doc = (ModelDoc2)sw.NewDocument(sw.GetUserPreferenceStringValue(8), 0, 0, 0);
        doc.Extension.SetUserPreferenceInteger((int)swUserPreferenceIntegerValue_e.swUnitSystem, 0,
                                               (int)swUnitSystem_e.swUnitSystem_MMGS);
        doc.SketchManager.AddToDB = true;
        return doc;
    }

    static Feature lastSketch;
    static void Sketch(ModelDoc2 doc, string plane, Action<SketchManager> draw)
    {
        doc.ClearSelection2(true);
        doc.Extension.SelectByID2(plane, "PLANE", 0, 0, 0, false, 0, null, 0);
        doc.SketchManager.InsertSketch(true);
        var sk = (Sketch)doc.SketchManager.ActiveSketch;
        draw(doc.SketchManager);
        doc.ClearSelection2(true);
        foreach (SketchSegment s in (object[])sk.GetSketchSegments()) s.Select4(true, null);
        doc.SketchAddConstraints("sgFIXED");                      // fully defined
        doc.ClearSelection2(true);
        doc.SketchManager.InsertSketch(true);
        lastSketch = (Feature)sk;
    }

    static void Extrude(ModelDoc2 doc, double d, string name)
    {
        doc.ClearSelection2(true);
        lastSketch.Select2(false, 0);
        var f = (Feature)doc.FeatureManager.FeatureExtrusion2(true, false, false, 0, 0, d, 0,
            false, false, false, false, 0, 0, false, false, false, false, true, false, true, 0, 0, false);
        if (f == null) throw new Exception("extrude failed: " + name);
        lastSketch.Name = name + " sketch"; f.Name = name;
    }

    static void CutThrough(ModelDoc2 doc, string name)
    {
        doc.ClearSelection2(true);
        lastSketch.Select2(false, 0);
        var f = (Feature)doc.FeatureManager.FeatureCut4(false, false, false, 1, 1, 0.01, 0.01,
            false, false, false, false, 0, 0, false, false, false, false, false, true, true,
            true, true, false, 0, 0, false, false);
        if (f == null) throw new Exception("cut failed: " + name);
        lastSketch.Name = name + " sketch"; f.Name = name;
    }

    // ------------------------------------------------------------------ assembly
    static void Asm(SldWorks sw, string[] spec)
    {
        string asmPath = Dir + (System.Environment.GetEnvironmentVariable("INTERIOR_ASM") ?? "CubeSat_3U_v2.SLDASM");
        var openAsm = (ModelDoc2)sw.GetOpenDocumentByName(asmPath);
        if (openAsm != null) sw.CloseDoc(openAsm.GetTitle());
        if (File.Exists(asmPath)) File.Delete(asmPath);
        var place = new List<Tuple<string, double[], double[], string>>();
        Func<string, double[]> nums = s => s.Split(',').Select(D).ToArray();
        // structure (same transforms as Assembly.cs), top end plate flipped so its countersinks face out
        place.Add(Tuple.Create("side_frame.SLDPRT", nums("1,0,0,0,1,0,0,0,1"), nums("0,0,0"), "side frame -X"));
        place.Add(Tuple.Create("side_frame.SLDPRT", nums("-1,0,0,0,-1,0,0,0,1"), nums("100,100,0"), "side frame +X"));
        place.Add(Tuple.Create("shear_panel.SLDPRT", nums("1,0,0,0,1,0,0,0,1"), nums("1.5,1.5,7"), "shear panel -Y"));
        place.Add(Tuple.Create("shear_panel.SLDPRT", nums("1,0,0,0,1,0,0,0,1"), nums("1.5,97.5,7"), "shear panel +Y"));
        place.Add(Tuple.Create("end_plate.SLDPRT", nums("1,0,0,0,1,0,0,0,1"), nums("1.5,2.5,7"), "end plate -Z"));
        place.Add(Tuple.Create("end_plate_top.SLDPRT", nums("-1,0,0,0,1,0,0,0,-1"), nums("98.5,2.5,333.5"), "end plate +Z"));
        foreach (var line in spec.Where(l => l.StartsWith("COMP|")))
        {
            var f = line.Split('|');
            place.Add(Tuple.Create(f[1], nums(f[2]), nums(f[3]), f[4]));
        }
        int e = 0, w = 0;
        foreach (var file in place.Select(p => p.Item1).Distinct())
            if (sw.GetOpenDocumentByName(Dir + file) == null) sw.OpenDoc6(Dir + file, 1, 1, "", ref e, ref w);
        var doc = (ModelDoc2)sw.NewDocument(sw.GetUserPreferenceStringValue(9), 0, 0, 0);
        doc.Extension.SetUserPreferenceInteger((int)swUserPreferenceIntegerValue_e.swUnitSystem, 0,
                                               (int)swUnitSystem_e.swUnitSystem_MMGS);
        var asm = (AssemblyDoc)doc;
        var mu = (MathUtility)sw.GetMathUtility();
        var comps = new List<Component2>();
        int bad = 0;
        foreach (var p in place)
        {
            var c = (Component2)asm.AddComponent5(Dir + p.Item1, 0, "", false, "", 0, 0, 0);
            if (c == null) throw new Exception("could not add " + p.Item1);
            double[] R = p.Item2, t = p.Item3;
            // SOLIDWORKS stores the rotation for row vectors (p' = p R + t): pass R transposed
            c.Transform2 = (MathTransform)mu.CreateTransform(new double[] {
                R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8],
                t[0] / 1000, t[1] / 1000, t[2] / 1000, 1, 0, 0, 0 });
            // check: the transformed local origin and local z axis must land where intended
            var tr = (MathTransform)c.Transform2;
            double[] o = (double[])((MathPoint)((MathPoint)mu.CreatePoint(new double[] { 0, 0, 0 })).MultiplyTransform(tr)).ArrayData;
            double[] z = (double[])((MathVector)((MathVector)mu.CreateVector(new double[] { 0, 0, 1 })).MultiplyTransform(tr)).ArrayData;
            double err = Math.Abs(o[0] * 1000 - t[0]) + Math.Abs(o[1] * 1000 - t[1]) + Math.Abs(o[2] * 1000 - t[2])
                       + Math.Abs(z[0] - R[2]) + Math.Abs(z[1] - R[5]) + Math.Abs(z[2] - R[8]);
            if (err > 1e-6) { bad++; Console.WriteLine("PLACEMENT ERROR " + c.Name2 + " " + err); }
            comps.Add(c);
        }
        doc.ClearSelection2(true);
        foreach (var c in comps) c.Select4(true, null, false);
        asm.FixComponent();
        doc.ClearSelection2(true);
        doc.EditRebuild3();
        Console.WriteLine("components: " + comps.Count + ", placement errors: " + bad);

        var mp = (MassProperty)doc.Extension.CreateMassProperty();
        double[] cm = (double[])mp.CenterOfMass;
        double[] I = (double[])mp.GetMomentOfInertia(0);
        Console.WriteLine(string.Format("total mass {0:F2} g, CoM ({1:F2}, {2:F2}, {3:F2}) mm -> offset from centre ({4:+0.00;-0.00}, {5:+0.00;-0.00}, {6:+0.00;-0.00}) mm",
            mp.Mass * 1e3, cm[0] * 1e3, cm[1] * 1e3, cm[2] * 1e3, cm[0] * 1e3 - 50, cm[1] * 1e3 - 50, cm[2] * 1e3 - 170.25));
        Console.WriteLine(string.Format("inertia about CoM: Ixx {0:F5}  Iyy {1:F5}  Izz {2:F5}  (Ixy {3:E2} Ixz {4:E2} Iyz {5:E2}) kg m^2",
            I[0], I[4], I[8], I[1], I[2], I[5]));

        var idm = asm.InterferenceDetectionManager;
        idm.TreatCoincidenceAsInterference = false;
        idm.TreatSubAssembliesAsComponents = true;
        idm.IncludeMultibodyPartInterferences = true;
        object[] intf = (object[])idm.GetInterferences();
        Console.WriteLine("interferences: " + (intf == null ? 0 : intf.Length));
        if (intf != null)
            foreach (IInterference it in intf)
                Console.WriteLine(string.Format("  {0}: {1:F3} mm^3", string.Join(" + ",
                    ((object[])it.Components).Cast<Component2>().Select(c => c.Name2).ToArray()), it.Volume * 1e9));
        idm.Done();

        doc.ShowNamedView2("*Trimetric", -1);
        doc.ViewZoomtofit2();
        doc.Extension.SaveAs(asmPath, 0, 1, null, ref e, ref w);
        Console.WriteLine("saved " + asmPath + " (errors " + e + ", warnings " + w + ")");
    }
}
