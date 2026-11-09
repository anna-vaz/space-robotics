// Adds blind M2 tap-drill holes (dia 1.6, 5 mm deep) into the 4 rail feet of side_frame.SLDPRT,
// on the feet's inner end faces (z = 7 and z = 333.5) at x = 4.5, y = 5.5 / 94.5. They take the
// end-plate screws (end plate holes at 3 mm from its corner = box 1.5 + 3, 2.5 + 3).
// Refuses to touch the part if it has unsaved changes; saves only if all checks pass.
using System;
using System.IO;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

public static class FootTaps
{
    const string PartPath = @"D:\cad free projects\CubeSat 3U\side_frame.SLDPRT";
    const double R = 0.0008, DEPTH = 0.005;

    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        var open = (ModelDoc2)sw.GetOpenDocumentByName(PartPath);
        if (open != null && open.GetSaveFlag()) { Console.WriteLine("side_frame has UNSAVED changes - nothing done"); return; }
        int e = 0, w = 0;
        var doc = open ?? (ModelDoc2)sw.OpenDoc6(PartPath, 1, 0, "", ref e, ref w);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var part = (PartDoc)doc;
        int before = Holes(part, false);
        if (FootHoles(part) > 0) { Console.WriteLine("foot taps already present - nothing done"); return; }
        double m0 = ((MassProperty)doc.Extension.CreateMassProperty()).Mass;
        Console.WriteLine(string.Format("before: mass {0:F3} g, r=0.8 hole faces {1}", m0 * 1e3, before));

        var mu = (MathUtility)sw.GetMathUtility();
        foreach (double zf in new[] { 0.007, 0.3335 })
        {
            Face2 face = null;
            foreach (Body2 b in (object[])part.GetBodies2(0, true))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    if (Math.Abs(bx[2] - zf) < 1e-7 && Math.Abs(bx[5] - zf) < 1e-7 && bx[4] < 0.0101) face = f;
                }
            if (face == null) throw new Exception("foot end face at z = " + zf + " not found");
            // any face at that z: the two feet are separate faces; select both
            doc.ClearSelection2(true);
            foreach (Body2 b in (object[])part.GetBodies2(0, true))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    if (Math.Abs(bx[2] - zf) < 1e-7 && Math.Abs(bx[5] - zf) < 1e-7 && bx[4] < 0.0101) { face = f; break; }
                }
            ((Entity)face).Select4(false, null);
            doc.SketchManager.InsertSketch(true);
            var sk = (Sketch)doc.SketchManager.ActiveSketch;
            var toSk = (MathTransform)sk.ModelToSketchTransform;
            doc.SketchManager.AddToDB = true;
            foreach (double y in new[] { 0.0055, 0.0945 })
            {
                double[] q = (double[])((MathPoint)((MathPoint)mu.CreatePoint(new double[] { 0.0045, y, zf }))
                                         .MultiplyTransform(toSk)).ArrayData;
                var c = (SketchSegment)doc.SketchManager.CreateCircleByRadius(q[0], q[1], 0, R);
                doc.ClearSelection2(true);
                c.Select4(false, null);
                doc.SketchAddConstraints("sgFIXED");
            }
            doc.SketchManager.AddToDB = false;
            doc.ClearSelection2(true);
            doc.SketchManager.InsertSketch(true);
            var skf = (Feature)sk;
            string end = zf < 0.1 ? "z0 end" : "z340 end";
            skf.Name = "Foot taps sketch " + end;
            Feature cut = null;
            int n0 = FootHoles(part);
            foreach (bool flip in new[] { false, true })
            {
                doc.ClearSelection2(true);
                skf.Select2(false, 0);
                cut = (Feature)doc.FeatureManager.FeatureCut4(true, flip, false, 0, 0, DEPTH, DEPTH,
                    false, false, false, false, 0, 0, false, false, false, false, false, true, true,
                    true, true, false, 0, 0, false, false);
                if (cut != null && FootHoles(part) > n0) break;
                if (cut != null) { doc.EditUndo2(1); cut = null; }
            }
            if (cut == null) throw new Exception("blind cut failed at " + end + " - part NOT saved");
            cut.Name = "M2 Foot Taps (1.6 drill, 5 deep) " + end;
        }

        double m1 = ((MassProperty)doc.Extension.CreateMassProperty()).Mass;
        Holes(part, true);
        Console.WriteLine(string.Format("after: mass {0:F3} g (change {1:F3} g), foot hole faces {2}",
            m1 * 1e3, (m1 - m0) * 1e3, FootHoles(part)));
        if (FootHoles(part) != 4) throw new Exception("expected 4 foot holes - part NOT saved");
        doc.Save3(1, ref e, ref w);
        Console.WriteLine("saved (errors " + e + ", warnings " + w + ")");
    }

    // r = 0.8 mm cylinders running along z (the foot taps)
    static int FootHoles(PartDoc part)
    {
        int n = 0;
        foreach (Body2 b in (object[])part.GetBodies2(0, true))
            foreach (Face2 f in (object[])b.GetFaces())
            {
                var s = (Surface)f.GetSurface();
                if (!s.IsCylinder()) continue;
                double[] p = (double[])s.CylinderParams;
                if (Math.Abs(p[6] - R) < 1e-7 && Math.Abs(Math.Abs(p[5]) - 1) < 1e-6) n++;
            }
        return n;
    }

    static int Holes(PartDoc part, bool print)
    {
        int n = 0;
        foreach (Body2 b in (object[])part.GetBodies2(0, true))
            foreach (Face2 f in (object[])b.GetFaces())
            {
                var s = (Surface)f.GetSurface();
                if (!s.IsCylinder()) continue;
                double[] p = (double[])s.CylinderParams;
                if (Math.Abs(p[6] - R) > 1e-7) continue;
                n++;
                if (print && Math.Abs(Math.Abs(p[5]) - 1) < 1e-6)
                {
                    double[] bx = (double[])f.GetBox();
                    Console.WriteLine(string.Format("  foot tap x {0:F3} y {1:F3}  z {2:F2}..{3:F2} mm",
                        p[0] * 1e3, p[1] * 1e3, bx[2] * 1e3, bx[5] * 1e3));
                }
            }
        return n;
    }
}
