// Probe: find the accepted data formats for SetTimeOrFrequencyCurve2 and SetDampingRatios on a random-vibration
// study (no mesh, no solve). Prints error codes and read-back values for each candidate format.
using System;
using System.Linq;
using System.Collections.Generic;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class RVProbe
{
    public static void Run(string outDir)
    {
        var sw = SwSim.Connect(Console.Out);
        int e = 0, w = 0;
        string asm = @"D:\cad free projects\CubeSat 3U\CubeSat_3U_v4.SLDASM";
        var doc = (ModelDoc2)sw.GetOpenDocumentByName(asm) ?? (ModelDoc2)sw.OpenDoc6(asm, 2, 0, "", ref e, ref w);
        var cw = SwSim.Simulation(sw, Console.Out);
        var sm = cw.ActiveDoc.StudyManager;
        int err;
        CWStudy st = sm.CreateNewStudy3("RVProbe_" + DateTime.Now.Ticks % 100000, 8, 2, out err);
        var o = st.DynamicStudyOptions;
        int u; o.GetRandomVibrationFrequencyUnits2(out u); Console.WriteLine("freq units now " + u);
        foreach (int uu in new[] { 0, 1, 2 }) Console.WriteLine("  set units " + uu + " err " + o.SetRandomVibrationFrequencyUnits2(uu));
        var dmp = st.DampingOptions; dmp.DampingType = 0; dmp.ComputeFromMaterialDamping2 = false; Console.WriteLine("material damping now " + dmp.ComputeFromMaterialDamping2);
        Console.WriteLine("damping now: " + Fmt(dmp.GetDampingRatios()));
        var cands = new Dictionary<string, double[]> {
            { "1 row [1,1000,0.05]", new double[] { 1, 1000, 0.05 } },
            { "1 row [1,0.05]", new double[] { 1, 0.05 } },
            { "2 rows [1,0.05,1000,0.05]", new double[] { 1, 0.05, 1000, 0.05 } },
            { "1 row [1,1000,5]", new double[] { 1, 1000, 5 } },
            { "2 rows [1,5,1000,5]", new double[] { 1, 5, 1000, 5 } } };
        foreach (var kv in cands)
        {
            int rows = kv.Key.StartsWith("2") ? 2 : 1;
            int r = dmp.SetDampingRatios(rows, kv.Value);
            int r2 = dmp.SetDampingRatios(rows, kv.Value.Cast<object>().ToArray());
            Console.WriteLine("    (object[] version err " + r2 + ")");
            Console.WriteLine("  damping " + kv.Key + ": err " + r + " -> " + Fmt(dmp.GetDampingRatios()));
        }
        var front = (Feature)((AssemblyDoc)doc).FeatureByName("Front Plane");
        // need a fixture first
        var comps = ((object[])((AssemblyDoc)doc).GetComponents(true)).Cast<Component2>().ToList();
        var ends = new List<object>();
        foreach (var c in comps.Where(c => c.Name2.StartsWith("side_frame")))
            foreach (Body2 b in (object[])c.GetBodies2(0))
                foreach (Face2 f in (object[])b.GetFaces())
                {
                    if (!((Surface)f.GetSurface()).IsPlane()) continue;
                    double[] bx = (double[])f.GetBox();
                    if (Math.Abs(bx[2]) < 1e-7 && Math.Abs(bx[5]) < 1e-7) ends.Add(f);
                }
        st.LoadsAndRestraintsManager.AddRestraint(0, ends.ToArray(), null, out err);
        var bex = st.LoadsAndRestraintsManager.AddUniformBaseExcitation2(2, front, 3, false, 0, true, 1, false, 0, out err);
        Console.WriteLine("base exc err " + err + ", curve now " + Fmt(bex.GetTimeOrFrequencyCurve()));
        var variants = new List<KeyValuePair<string, object>>();
        variants.Add(new KeyValuePair<string, object>("object[] int n", new object[] { 3, 20.0, 0.026, 50.0, 0.16, 800.0, 0.16 }));
        variants.Add(new KeyValuePair<string, object>("double[,] 3x2", new double[,] { { 20, 0.026 }, { 50, 0.16 }, { 800, 0.16 } }));
        variants.Add(new KeyValuePair<string, object>("object[,] 3x2", new object[,] { { 20.0, 0.026 }, { 50.0, 0.16 }, { 800.0, 0.16 } }));
        variants.Add(new KeyValuePair<string, object>("double[] n x y", new double[] { 3, 20, 0.026, 50, 0.16, 800, 0.16 }));
        variants.Add(new KeyValuePair<string, object>("int[]-ish object n=3 then floats(4pt)", new object[] { 4, 20.0, 0.026, 50.0, 0.16, 800.0, 0.16, 2000.0, 0.026 }));
        foreach (var kv in variants)
        {
            int ce = -9; bool ok = false; string ex = "";
            bex.BaseExcitationBeginEdit();
            try { ok = bex.SetTimeOrFrequencyCurve2(kv.Value, out ce); } catch (Exception x) { ex = x.Message; }
            int ee = bex.BaseExcitationEndEdit();
            bex.BaseExcitationBeginEdit(); string rb = Fmt(bex.GetTimeOrFrequencyCurve()); bex.BaseExcitationEndEdit();
            Console.WriteLine("  curve " + kv.Key + ": ok " + ok + " err " + ce + " end " + ee + " " + ex + " -> " + rb);
            int ce2 = -9; bool ok2 = false;
            try { ok2 = bex.SetTimeOrFrequencyCurve2(kv.Value, out ce2); } catch (Exception x) { ex = x.Message; }
            Console.WriteLine("     no-edit: ok " + ok2 + " err " + ce2 + " -> " + Fmt(bex.GetTimeOrFrequencyCurve()));
        }
        // read curve back through a fresh LoadsAndRestraints lookup
        var lr2 = st.LoadsAndRestraintsManager;
        Console.WriteLine("L&R count " + lr2.Count);
        for (int i = 0; i < lr2.Count; i++)
        {
            int ec; object it = lr2.GetLoadsAndRestraints(i, out ec);
            var b2 = it as CWBaseExcitation;
            if (b2 != null) Console.WriteLine("  item " + i + " base excitation curve: " + Fmt(b2.GetTimeOrFrequencyCurve()));
        }
    }
    static string Fmt(object o)
    {
        if (o == null || o is DBNull) return "null";
        return string.Join(", ", ((Array)o).Cast<object>().Select(x => Convert.ToDouble(x).ToString("G4")).ToArray());
    }
}
