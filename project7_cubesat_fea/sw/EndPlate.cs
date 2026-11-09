// Builds CAD part 3, the end plate: 97 x 95 x 2 mm 6061-T6, 4x M2 clearance (2.4) foot-screw
// holes at 3 mm from the edges, 4x M3 clearance (3.4) PC/104 stack-rod holes
// (first at 5.635 / 7.495, spacing 85.73 x 80.01). Origin at the plate corner, plate on the
// Front Plane, thickness along +z (same convention as the side frame and shear panel).
using System;
using System.IO;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

public static class EndPlate
{
    const string PartPath = @"D:\cad free projects\CubeSat 3U\end_plate.SLDPRT";

    public static void Run(string outDir)
    {
        var swc = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        foreach (object o in (object[])swc.GetDocuments() ?? new object[0])
        {   // discard an unsaved leftover from a failed run
            var d = (ModelDoc2)o;
            if (d.GetPathName() == "" && d.GetTitle().StartsWith("Part")) { swc.CloseDoc(d.GetTitle()); Console.WriteLine("closed unsaved " + d.GetTitle()); }
        }
        if (File.Exists(PartPath)) { Console.WriteLine("end_plate.SLDPRT already exists - nothing done"); return; }
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        bool oldInput = sw.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swInputDimValOnCreate);
        sw.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swInputDimValOnCreate, false);
        try { Build(sw); }
        finally { sw.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swInputDimValOnCreate, oldInput); }
    }

    static void Build(SldWorks sw)
    {
        const double A = 0.097, B = 0.095, T = 0.002;
        var doc = (ModelDoc2)sw.NewDocument(sw.GetUserPreferenceStringValue(8), 0, 0, 0);
        doc.Extension.SetUserPreferenceInteger((int)swUserPreferenceIntegerValue_e.swUnitSystem, 0,
                                               (int)swUnitSystem_e.swUnitSystem_MMGS);
        var sm = doc.SketchManager;
        sm.AddToDB = true;

        // ---- plate
        doc.Extension.SelectByID2("Front Plane", "PLANE", 0, 0, 0, false, 0, null, 0);
        sm.InsertSketch(true);
        object[] lines = (object[])sm.CreateCornerRectangle(0, 0, 0, A, B, 0);
        // corner on the origin, then width/height dimensions
        doc.ClearSelection2(true);
        doc.Extension.SelectByID2("Point1@Origin", "EXTSKETCHPOINT", 0, 0, 0, false, 0, null, 0);
        SketchPoint corner = FindPoint(sm, 0, 0);
        corner.Select4(true, null);
        doc.SketchAddConstraints("sgCOINCIDENT");
        foreach (SketchSegment s in lines)
        {
            var ln = (SketchLine)s;
            var p0 = (SketchPoint)ln.GetStartPoint2(); var p1 = (SketchPoint)ln.GetEndPoint2();
            bool horizontalAtBottom = Math.Abs(p0.Y) < 1e-9 && Math.Abs(p1.Y) < 1e-9;
            bool verticalAtLeft = Math.Abs(p0.X) < 1e-9 && Math.Abs(p1.X) < 1e-9;
            if (!horizontalAtBottom && !verticalAtLeft) continue;
            doc.ClearSelection2(true);
            s.Select4(false, null);
            if (horizontalAtBottom) doc.AddDimension2(A / 2, -0.008, 0);
            else doc.AddDimension2(-0.008, B / 2, 0);
        }
        doc.ClearSelection2(true);
        var plateSketch = (Sketch)sm.ActiveSketch;
        sm.InsertSketch(true);
        Console.WriteLine("plate sketch status (3 = fully defined): " + plateSketch.GetConstrainedStatus());
        ((Feature)plateSketch).Name = "Plate sketch";
        doc.ClearSelection2(true);
        ((Feature)plateSketch).Select2(false, 0);
        var plate = (Feature)doc.FeatureManager.FeatureExtrusion2(true, false, false, 0, 0, T, 0,
            false, false, false, false, 0, 0, false, false, false, false, true, false, true, 0, 0, false);
        plate.Name = "Plate";

        // ---- holes (fixed circles, cut through all)
        Holes(doc, sm, "M2 foot screw holes", 0.0012,
              new[] { 0.003, 0.094 }, new[] { 0.003, 0.092 });
        double x0 = 0.005635, y0 = 0.007495;
        Holes(doc, sm, "M3 stack rod holes", 0.0017,
              new[] { x0, x0 + 0.08573 }, new[] { y0, y0 + 0.08001 });

        ((PartDoc)doc).SetMaterialPropertyName2("", "SOLIDWORKS Materials", "6061-T6 (SS)");
        doc.EditRebuild3();
        doc.ShowNamedView2("*Isometric", -1);
        doc.ViewZoomtofit2();

        var mp = (MassProperty)doc.Extension.CreateMassProperty();
        double[] c = (double[])mp.CenterOfMass;
        double[] box = (double[])((PartDoc)doc).GetPartBox(true);
        Console.WriteLine(string.Format("mass {0:F3} g  volume {1:F1} mm^3  CoM ({2:F3}, {3:F3}, {4:F3}) mm",
            mp.Mass * 1e3, mp.Volume * 1e9, c[0] * 1e3, c[1] * 1e3, c[2] * 1e3));
        Console.WriteLine(string.Format("box x {0:F2}..{1:F2}  y {2:F2}..{3:F2}  z {4:F2}..{5:F2} mm",
            box[0] * 1e3, box[3] * 1e3, box[1] * 1e3, box[4] * 1e3, box[2] * 1e3, box[5] * 1e3));
        int n24 = 0, n34 = 0;
        foreach (Body2 b in (object[])((PartDoc)doc).GetBodies2(0, true))
            foreach (Face2 f in (object[])b.GetFaces())
            {
                var s = (Surface)f.GetSurface();
                if (!s.IsCylinder()) continue;
                double[] p = (double[])s.CylinderParams;
                if (Math.Abs(p[6] - 0.0012) < 1e-7) n24++;
                if (Math.Abs(p[6] - 0.0017) < 1e-7) n34++;
                Console.WriteLine(string.Format("  hole d {0:F2} at ({1:F3}, {2:F3})", 2 * p[6] * 1e3, p[0] * 1e3, p[1] * 1e3));
            }
        if (n24 != 4 || n34 != 4 || Math.Abs(mp.Mass - 0.04947) > 0.0001)
            throw new Exception("checks failed (holes " + n24 + "/" + n34 + ") - part NOT saved, left open for inspection");
        int e = 0, w = 0;
        doc.Extension.SaveAs(PartPath, 0, 1, null, ref e, ref w);
        Console.WriteLine("saved " + PartPath + " (errors " + e + ", warnings " + w + ")");
    }

    static SketchPoint FindPoint(SketchManager sm, double x, double y)
    {
        foreach (SketchPoint p in (object[])((Sketch)sm.ActiveSketch).GetSketchPoints2())
            if (Math.Abs(p.X - x) < 1e-9 && Math.Abs(p.Y - y) < 1e-9) return p;
        throw new Exception("sketch point not found");
    }

    static void Holes(ModelDoc2 doc, SketchManager sm, string name, double r, double[] xs, double[] ys)
    {
        doc.ClearSelection2(true);
        doc.Extension.SelectByID2("Front Plane", "PLANE", 0, 0, 0, false, 0, null, 0);
        sm.InsertSketch(true);
        foreach (double y in ys)
            foreach (double x in xs)
            {
                var c = (SketchSegment)sm.CreateCircleByRadius(x, y, 0, r);
                doc.ClearSelection2(true);
                c.Select4(false, null);
                doc.SketchAddConstraints("sgFIXED");
            }
        doc.ClearSelection2(true);
        var sk = (Sketch)sm.ActiveSketch;
        sm.InsertSketch(true);
        ((Feature)sk).Name = name + " sketch";
        Feature cut = null;
        foreach (bool flip in new[] { false, true })
        {
            doc.ClearSelection2(true);
            ((Feature)sk).Select2(false, 0);
            // two-direction Through All from the Front Plane (the plate's back face)
            cut = (Feature)doc.FeatureManager.FeatureCut4(false, flip, false, 1, 1, 0.01, 0.01,
                false, false, false, false, 0, 0, false, false, false, false, false, true, true,
                true, true, false, 0, 0, false, false);
            if (cut != null) break;
        }
        if (cut == null) throw new Exception("cut failed: " + name);
        cut.Name = name;
        Console.WriteLine(name + ": sketch status " + sk.GetConstrainedStatus());
    }
}
