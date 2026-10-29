// Step 0: (a) which Simulation study types this licence can create,
//         (b) solver benchmark: cantilever block, first modes vs beam theory.
using System;
using System.IO;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.cosworks;

public static class Step0
{
    public static void Run(string outDir)
    {
        Directory.CreateDirectory(outDir);
        using (var log = new StreamWriter(Path.Combine(outDir, "step0_log.txt")))
        {
            var tee = new Tee(log);
            SldWorks sw = SwSim.Connect(tee);
            CosmosWorks cw = SwSim.Simulation(sw, tee);

            // Benchmark part: 20 x 20 mm Al 6061-T6 bar, 200 mm long, fixed at z = 0.
            const double a = 0.020, L = 0.200;
            ModelDoc2 doc = SwSim.NewBlockPart(sw, a, a, L, "6061-T6 (SS)");
            string part = Path.Combine(outDir, "step0_cantilever.SLDPRT");
            int e = 0, w = 0;
            doc.Extension.SaveAs(part, 0, 1, null, ref e, ref w);
            tee.WriteLine("saved " + part + " (err " + e + ")");

            CWModelDoc cdoc = cw.ActiveDoc;
            CWStudyManager sm = cdoc.StudyManager;

            // (a) licence probe: try to create one study of each type.
            var probes = new object[][] {
                new object[] {"Static", 0, 0}, new object[] {"Frequency", 1, 0},
                new object[] {"Buckling", 2, 0}, new object[] {"Thermal", 3, 0},
                new object[] {"Fatigue", 7, 0}, new object[] {"DropTest", 6, 0},
                new object[] {"Topology", 13, 0}, new object[] {"NonlinearStatic", 5, 0},
                new object[] {"NonlinearDynamic", 5, 1},
                new object[] {"LinDyn_Transient", 8, 0}, new object[] {"LinDyn_Harmonic", 8, 1},
                new object[] {"LinDyn_Random", 8, 2}, new object[] {"LinDyn_ResponseSpectrum", 8, 3},
            };
            tee.WriteLine("== licence probe (CreateNewStudy3 error 0 = created)");
            foreach (var p in probes)
            {
                int err = -1;
                CWStudy s = null;
                try { s = sm.CreateNewStudy3("probe_" + p[0], (int)p[1], (int)p[2], out err); }
                catch (Exception ex) { tee.WriteLine("  " + p[0] + ": EXCEPTION " + ex.Message); continue; }
                tee.WriteLine(string.Format("  {0,-24} err={1} created={2}", p[0], err, s != null));
            }
            for (int i = sm.StudyCount - 1; i >= 0; i--)
            {
                CWStudy s = sm.GetStudy(i);
                if (s != null && s.Name.StartsWith("probe_")) sm.DeleteStudy(s.Name);
            }

            // (b) benchmark frequency study.
            int er;
            CWStudy st = sm.CreateNewStudy3("Benchmark_Frequency", 1, 0, out er);
            st.FrequencyStudyOptions.NoOfFrequencies = 6;
            object[] root = SwSim.FacesOnPlane(doc, 2, 0.0, 1e-9);   // z = 0 face
            tee.WriteLine("fixed faces found: " + root.Length);
            CWLoadsAndRestraintsManager lr = st.LoadsAndRestraintsManager;
            lr.AddRestraint(0, root, null, out er);
            tee.WriteLine("AddRestraint err: " + er);

            CWMesh mesh = st.Mesh;
            mesh.MesherType = 0;   // standard mesher
            mesh.Quality = 1;      // high = 10-node quadratic tetrahedra
            foreach (double h in new double[] { 4.0, 2.0 })
            {
                int me = st.CreateMesh(0, h, h * 0.05);       // mm
                int re = st.RunAnalysis();
                CWResults res = st.Results;
                int fe;
                object fr = res.GetResonantFrequencies(out fe);
                tee.WriteLine(string.Format("mesh h={0} mm: meshErr={1} nodes={2} elems={3} runErr={4}",
                    h, me, mesh.NodeCount, mesh.ElementCount, re));
                tee.WriteLine("  resonant frequencies (raw): " + SwSim.Join(fr));
                object mp = res.GetMassParticipation(out fe);
                tee.WriteLine("  mass participation (raw): " + SwSim.Join(mp));
            }
            doc.Save3(1, ref e, ref w);
            tee.WriteLine("done");
        }
    }

    class Tee : TextWriter
    {
        readonly TextWriter f;
        public Tee(TextWriter f) { this.f = f; }
        public override System.Text.Encoding Encoding { get { return System.Text.Encoding.UTF8; } }
        public override void WriteLine(string s) { f.WriteLine(s); f.Flush(); Console.WriteLine(s); }
    }
}
