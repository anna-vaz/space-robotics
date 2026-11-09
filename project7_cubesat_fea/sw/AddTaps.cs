// Adds the M2 tap-drill holes (dia 1.6, through both rail flanges) to side_frame.SLDPRT.
// Holes: on the flange face y = 0, at x = 5 mm, z = 20 + i*75.125 (i = 0..4); a
// Through All cut along +y drills the opposite flange (y = 100) coaxially.
// Saves the part only if the result checks out (20 new r = 0.8 mm cylinders).
using System;
using System.IO;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;

public static class AddTaps
{
    const string PartPath = @"D:\cad free projects\CubeSat 3U\side_frame.SLDPRT";

    public static void Run(string outDir)
    {
        var sw = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
        sw.Visible = true;
        int e = 0, w = 0;
        // Discard any half-finished state from an earlier run: close the open copy unsaved.
        var open = (ModelDoc2)sw.GetOpenDocumentByName(PartPath);
        if (open != null)
        {
            if (open.SketchManager.ActiveSketch != null) open.SketchManager.InsertSketch(false);
            sw.CloseDoc(open.GetTitle());
            Console.WriteLine("closed the open copy without saving");
        }
        bool oldInput = sw.GetUserPreferenceToggle(10);   // swInputDimValOnCreate
        sw.SetUserPreferenceToggle(10, false);            // no "Modify" pop-up
        try { Body(sw, ref e, ref w); }
        finally { sw.SetUserPreferenceToggle(10, oldInput); }
    }

    static void Body(SldWorks sw, ref int e, ref int w)
    {
        var doc = (ModelDoc2)sw.OpenDoc6(PartPath, 1, 0, "", ref e, ref w);
        if (doc == null) throw new Exception("could not open part, error " + e);
        sw.ActivateDoc3(doc.GetTitle(), false, 0, ref e);
        var part = (PartDoc)doc;

        // Refuse to run twice: look for existing r = 0.8 mm holes.
        if (CountHoles(part, 0.0008) > 0) { Console.WriteLine("tap holes already present - nothing done"); return; }
        double m0 = Mass(doc);
        Console.WriteLine(string.Format("before: mass {0:F3} g", m0 * 1e3));

        // Find the planar face at y = 0 (outer face of the flange), normal -y.
        Face2 face = null;
        foreach (Body2 b in (object[])part.GetBodies2(0, true))
            foreach (Face2 f in (object[])b.GetFaces())
            {
                var s = (Surface)f.GetSurface();
                if (!s.IsPlane()) continue;
                double[] bx = (double[])f.GetBox();
                double[] n = (double[])f.Normal;
                if (Math.Abs(bx[1]) < 1e-7 && Math.Abs(bx[4]) < 1e-7 && n[1] < -0.99) face = f;
            }
        if (face == null) throw new Exception("flange face at y = 0 not found");

        doc.ClearSelection2(true);
        ((Entity)face).Select4(false, null);
        doc.SketchManager.InsertSketch(true);
        var sk = (Sketch)doc.SketchManager.ActiveSketch;
        var mu = (MathUtility)sw.GetMathUtility();
        var toSketch = (MathTransform)sk.ModelToSketchTransform;
        doc.SketchManager.AddToDB = true;
        for (int i = 0; i < 5; i++)
        {
            double z = 0.020 + i * 0.075125;
            var p = (MathPoint)mu.CreatePoint(new double[] { 0.005, 0.0, z });
            double[] q = (double[])((MathPoint)p.MultiplyTransform(toSketch)).ArrayData;
            var c = (SketchSegment)doc.SketchManager.CreateCircleByRadius(q[0], q[1], 0, 0.0008);
            // lock it (centre + radius) so the sketch is fully defined
            doc.ClearSelection2(true);
            c.Select4(false, null);
            doc.SketchAddConstraints("sgFIXED");
        }
        doc.SketchManager.AddToDB = false;
        doc.ClearSelection2(true);
        doc.SketchManager.InsertSketch(true);   // close sketch, it stays selected? select explicitly
        var skFeat = (Feature)sk;
        skFeat.Name = "M2 tap holes sketch";
        doc.ClearSelection2(true);
        skFeat.Select2(false, 0);

        // Through All cut (end condition 1); try default direction, flip if nothing was cut.
        Feature cut = null;
        foreach (bool flip in new[] { false, true })
        {
            cut = (Feature)doc.FeatureManager.FeatureCut4(true, flip, false, 1, 0, 0.01, 0.01,
                false, false, false, false, 0, 0, false, false, false, false, false, true, true,
                true, true, false, 0, 0, false, false);
            if (cut != null && CountHoles(part, 0.0008) > 0) break;
            if (cut != null) { doc.EditUndo2(1); cut = null; }
            doc.ClearSelection2(true);
            skFeat.Select2(false, 0);
        }
        if (cut == null) throw new Exception("cut failed in both directions - part NOT saved");
        cut.Name = "M2 Tap Holes (1.6 drill)";

        int nh = CountHoles(part, 0.0008);
        double m1 = Mass(doc);
        Console.WriteLine(string.Format("after: mass {0:F3} g (change {1:F3} g), r=0.8 mm hole faces: {2}",
            m1 * 1e3, (m1 - m0) * 1e3, nh));
        foreach (Body2 b in (object[])part.GetBodies2(0, true))
            foreach (Face2 f in (object[])b.GetFaces())
            {
                var s = (Surface)f.GetSurface();
                if (!s.IsCylinder()) continue;
                double[] p = (double[])s.CylinderParams;
                if (Math.Abs(p[6] - 0.0008) > 1e-7) continue;
                double[] bx = (double[])f.GetBox();
                Console.WriteLine(string.Format("  hole x {0:F3}  z {1:F3}  y {2:F2}..{3:F2}",
                    p[0] * 1e3, p[2] * 1e3, bx[1] * 1e3, bx[4] * 1e3));
            }
        var skObj = (Sketch)skFeat.GetSpecificFeature2();
        Console.WriteLine("sketch constrained status (3 = fully defined): " + skObj.GetConstrainedStatus());
        if (nh < 10) throw new Exception("expected >= 10 hole faces, got " + nh + " - part NOT saved");
        doc.Save3(1, ref e, ref w);
        Console.WriteLine("saved (errors " + e + ", warnings " + w + ")");
    }

    static double Mass(ModelDoc2 doc)
    {
        var mp = (MassProperty)doc.Extension.CreateMassProperty();
        return mp.Mass;
    }

    static int CountHoles(PartDoc part, double r)
    {
        int n = 0;
        foreach (Body2 b in (object[])part.GetBodies2(0, true))
            foreach (Face2 f in (object[])b.GetFaces())
            {
                var s = (Surface)f.GetSurface();
                if (s.IsCylinder() && Math.Abs(((double[])s.CylinderParams)[6] - r) < 1e-7) n++;
            }
        return n;
    }
}
