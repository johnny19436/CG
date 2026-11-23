import type { DesignSystem, Page, SlideMeta } from '@open-slide/core';
import type { ReactNode } from 'react';

export const design: DesignSystem = {
  palette: { bg: '#08111f', text: '#eaf2ff', accent: '#38bdf8' },
  fonts: {
    display: 'Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif',
    body: 'Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif',
  },
  typeScale: { hero: 156, body: 34 },
  radius: 18,
};

const muted = '#94a3b8';
const panel = '#0f1b2d';
const panel2 = '#13233a';
const line = '#233955';
const warm = '#f59e0b';
const green = '#34d399';
const red = '#fb7185';
const violet = '#a78bfa';

const fill = {
  width: '100%',
  height: '100%',
  background: 'var(--osd-bg)',
  color: 'var(--osd-text)',
  fontFamily: 'var(--osd-font-body)',
  position: 'relative',
  overflow: 'hidden',
} as const;

type ShellProps = {
  eyebrow?: string;
  title: string;
  children: ReactNode;
};

const GridBackground = () => (
  <div
    style={{
      position: 'absolute',
      inset: 0,
      backgroundImage:
        'linear-gradient(rgba(56,189,248,0.08) 1px, transparent 1px), linear-gradient(90deg, rgba(56,189,248,0.08) 1px, transparent 1px)',
      backgroundSize: '80px 80px',
      maskImage: 'radial-gradient(circle at 70% 35%, black 0%, transparent 72%)',
      opacity: 0.9,
    }}
  />
);

const PageShell = ({ eyebrow, title, children }: ShellProps) => (
  <div style={{ ...fill, padding: '108px 112px 68px' }}>
    <GridBackground />
    <div style={{ position: 'relative', zIndex: 1 }}>
      {eyebrow ? (
        <div style={{ color: 'var(--osd-accent)', fontSize: 52, fontWeight: 900, letterSpacing: '0.08em', textTransform: 'uppercase' }}>
          {eyebrow}
        </div>
      ) : null}
      <p
        style={{ fontFamily: 'var(--osd-font-display)', fontSize: 60, lineHeight: 1.14, margin: eyebrow ? '20px 0 46px' : '0 0 46px', maxWidth: 1360, color: '#ffffff' }}
      >
        {title}
      </p>
      {children}
    </div>
  </div>
);

type CardProps = {
  title: string;
  body: string;
  accent?: string;
};

const Card = ({ title, body, accent = 'var(--osd-accent)' }: CardProps) => (
  <div
    style={{
      background: panel,
      border: `1px solid ${line}`,
      borderRadius: 'var(--osd-radius)',
      padding: 34,
      minHeight: 230,
      boxShadow: '0 24px 70px rgba(0,0,0,0.22)',
    }}
  >
    <div style={{ width: 54, height: 6, borderRadius: 999, background: accent, marginBottom: 28 }} />
    <h3 style={{ fontSize: 34, lineHeight: 1.15, margin: '0 0 18px', color: '#ffffff' }}>{title}</h3>
    <p style={{ fontSize: 27, lineHeight: 1.45, margin: 0, color: muted }}>{body}</p>
  </div>
);

const BulletList = ({ items }: { items: string[] }) => (
  <div style={{ display: 'grid', gap: 24 }}>
    {items.map((item, index) => (
      <div key={item} style={{ display: 'grid', gridTemplateColumns: '56px 1fr', alignItems: 'start', gap: 22 }}>
        <div
          style={{
            width: 46,
            height: 46,
            borderRadius: 14,
            background: 'rgba(56,189,248,0.14)',
            color: 'var(--osd-accent)',
            display: 'grid',
            placeItems: 'center',
            fontSize: 24,
            fontWeight: 900,
          }}
        >
          {index + 1}
        </div>
        <div style={{ fontSize: 34, lineHeight: 1.38, color: '#dbeafe' }}>{item}</div>
      </div>
    ))}
  </div>
);

const CodeBox = ({ children }: { children: ReactNode }) => (
  <div
    style={{
      background: '#050b14',
      border: `1px solid ${line}`,
      borderRadius: 22,
      padding: '34px 40px',
      fontFamily: '"Cascadia Code", "SFMono-Regular", Consolas, monospace',
      fontSize: 34,
      lineHeight: 1.55,
      color: '#d1fae5',
      whiteSpace: 'pre-wrap',
      boxShadow: 'inset 0 0 0 1px rgba(255,255,255,0.02)',
    }}
  >
    {children}
  </div>
);

const FlowDiagram = () => {
  const steps = [
    ['Advect', green],
    ['Diffuse', violet],
    ['Add force', warm],
    ['Project', 'var(--osd-accent)'],
  ];

  return (
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 18, alignItems: 'center' }}>
      {steps.map(([label, color], index) => (
        <div key={label} style={{ display: 'grid', gridTemplateColumns: index === steps.length - 1 ? '1fr' : '1fr 28px', gap: 12, alignItems: 'center' }}>
          <div style={{ background: panel, border: `1px solid ${line}`, borderRadius: 18, padding: '34px 20px', textAlign: 'center' }}>
            <div style={{ color, fontSize: 24, fontWeight: 900, marginBottom: 12 }}>0{index + 1}</div>
            <div style={{ color: '#ffffff', fontSize: 32, fontWeight: 850 }}>{label}</div>
          </div>
          {index === steps.length - 1 ? null : <div style={{ color: muted, fontSize: 30, fontWeight: 900 }}>→</div>}
        </div>
      ))}
    </div>
  );
};

const GridSketch = () => (
  <div style={{ background: panel, border: `1px solid ${line}`, borderRadius: 22, padding: 34 }}>
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(8, 1fr)', gap: 6 }}>
      {Array.from({ length: 40 }).map((_, index) => {
        const active = [18, 19, 20, 26, 27].includes(index);
        return (
          <div
            key={index}
            style={{
              aspectRatio: '1 / 1',
              borderRadius: 8,
              background: active ? 'rgba(56,189,248,0.8)' : 'rgba(148,163,184,0.12)',
              border: `1px solid ${active ? 'rgba(56,189,248,0.95)' : line}`,
            }}
          />
        );
      })}
    </div>
    <div style={{ marginTop: 22, fontSize: 26, lineHeight: 1.35, color: muted }}>
      Fixed cells hold velocity, pressure, divergence, and dye; kernels update the same index layout.
    </div>
  </div>
);

const ComparisonTable = () => (
  <div style={{ background: panel, border: `1px solid ${line}`, borderRadius: 22, overflow: 'hidden' }}>
    {[
      ['GPU Gems', 'Textures', 'Fragment per cell', 'Render target writes'],
      ['This project', 'float arrays', 'CUDA thread per cell', 'Device buffer writes'],
    ].map((row, rowIndex) => (
      <div key={row[0]} style={{ display: 'grid', gridTemplateColumns: '0.7fr 1fr 1fr 1fr', borderTop: rowIndex ? `1px solid ${line}` : 'none' }}>
        {row.map((cell, colIndex) => (
          <div
            key={cell}
            style={{
              padding: '26px 28px',
              fontSize: colIndex === 0 ? 30 : 26,
              lineHeight: 1.25,
              color: colIndex === 0 ? (rowIndex ? 'var(--osd-accent)' : warm) : '#dbeafe',
              fontWeight: colIndex === 0 ? 900 : 650,
              borderLeft: colIndex ? `1px solid ${line}` : 'none',
              background: colIndex === 0 ? 'rgba(255,255,255,0.03)' : 'transparent',
            }}
          >
            {cell}
          </div>
        ))}
      </div>
    ))}
  </div>
);

const FieldMatrix = () => {
  const fields = [
    ['vx, vy', 'vector', 'advect velocity and dye', green],
    ['pressure', 'scalar', 'projection solve', warm],
    ['divergence', 'scalar', 'inflow / outflow error', red],
    ['dye', 'visual', 'passive tracer', violet],
  ];

  return (
    <div style={{ display: 'grid', gridTemplateColumns: '0.9fr 0.8fr 1.3fr', border: `1px solid ${line}`, borderRadius: 22, overflow: 'hidden', background: panel }}>
      {['Field', 'Type', 'Role'].map((header) => (
        <div key={header} style={{ padding: '22px 28px', color: muted, fontSize: 22, fontWeight: 900, letterSpacing: '0.12em', textTransform: 'uppercase', background: '#0b1728' }}>
          {header}
        </div>
      ))}
      {fields.flatMap(([name, type, role, color]) => [
        <div key={`${name}-name`} style={{ padding: '26px 28px', fontSize: 34, fontWeight: 900, color, borderTop: `1px solid ${line}` }}>{name}</div>,
        <div key={`${name}-type`} style={{ padding: '26px 28px', fontSize: 28, color: '#dbeafe', borderTop: `1px solid ${line}` }}>{type}</div>,
        <div key={`${name}-role`} style={{ padding: '26px 28px', fontSize: 28, color: '#dbeafe', borderTop: `1px solid ${line}` }}>{role}</div>,
      ])}
    </div>
  );
};

const ParallelDiagram = () => (
  <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 36 }}>
    <div style={{ background: panel, border: `1px solid ${line}`, borderRadius: 22, padding: 34 }}>
      <h3 style={{ margin: '0 0 26px', fontSize: 40 }}>CPU loop</h3>
      <div style={{ display: 'grid', gap: 12 }}>
        {['row 0', 'row 1', 'row 2', 'row 3'].map((label, index) => (
          <div key={label} style={{ display: 'grid', gridTemplateColumns: '90px 1fr', gap: 16, alignItems: 'center' }}>
            <div style={{ color: muted, fontSize: 22 }}>{label}</div>
            <div style={{ height: 34, borderRadius: 10, background: index === 1 ? 'rgba(56,189,248,0.72)' : 'rgba(148,163,184,0.16)' }} />
          </div>
        ))}
      </div>
      <p style={{ margin: '28px 0 0', color: muted, fontSize: 26 }}>Cells are visited one after another.</p>
    </div>
    <div style={{ background: panel, border: `1px solid ${line}`, borderRadius: 22, padding: 34 }}>
      <h3 style={{ margin: '0 0 26px', fontSize: 40 }}>
        {/* @slide-comment id="c-f2c37076" ts="2026-06-04T16:55:48.455Z" text="eyJub3RlIjoiaGkifQ" */}GPU kernel</h3>
      <div style={{ display: 'grid', gridTemplateColumns: 'repeat(8, 1fr)', gap: 9 }}>
        {Array.from({ length: 32 }).map((_, index) => (
          <div key={index} style={{ aspectRatio: '1 / 1', borderRadius: 9, background: index % 5 === 0 ? 'rgba(245,158,11,0.82)' : 'rgba(56,189,248,0.5)' }} />
        ))}
      </div>
      <p style={{ margin: '28px 0 0', color: muted, fontSize: 26 }}>Many threads update cells in parallel.</p>
    </div>
  </div>
);

const AdvectionVisual = () => (
  <div style={{ background: panel, border: `1px solid ${line}`, borderRadius: 22, padding: 30, minHeight: 370 }}>
    <svg viewBox="0 0 640 360" style={{ width: '100%', height: 330, display: 'block' }} role="img" aria-label="Semi-Lagrangian advection traces backward from the current cell and samples the previous field">
      <defs>
        <marker id="arrowHead" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
          <path d="M 0 0 L 10 5 L 0 10 z" fill="#38bdf8" />
        </marker>
      </defs>
      {Array.from({ length: 7 }).map((_, col) => (
        <line key={`v-${col}`} x1={70 + col * 78} y1="52" x2={70 + col * 78} y2="270" stroke="#233955" strokeWidth="2" />
      ))}
      {Array.from({ length: 4 }).map((_, row) => (
        <line key={`h-${row}`} x1="70" y1={52 + row * 72} x2="538" y2={52 + row * 72} stroke="#233955" strokeWidth="2" />
      ))}
      <circle cx="420" cy="124" r="39" fill="rgba(56,189,248,0.15)" stroke="#38bdf8" strokeWidth="6" />
      <circle cx="202" cy="198" r="36" fill="rgba(167,139,250,0.92)" />
      <path d="M 418 124 C 360 136, 288 166, 214 194" fill="none" stroke="#38bdf8" strokeWidth="8" strokeLinecap="round" markerEnd="url(#arrowHead)" />
      <text x="420" y="88" fill="#eaf2ff" fontSize="28" fontWeight="800" textAnchor="middle">current cell</text>
      <text x="202" y="254" fill="#c4b5fd" fontSize="28" fontWeight="800" textAnchor="middle">sample point</text>
      <text x="320" y="320" fill="#94a3b8" fontSize="25" fontWeight="650" textAnchor="middle">trace backward through velocity, then bilinear sample</text>
    </svg>
  </div>
);

const ProjectionVisual = () => (
  <div style={{ display: 'grid', gridTemplateColumns: '1fr 44px 1fr 44px 1fr', gap: 18, alignItems: 'center' }}>
    {[
      ['Divergence', 'measure imbalance', red],
      ['Pressure', 'solve Poisson equation', warm],
      ['Velocity', 'subtract gradient', green],
    ].map(([label, note, color], index) => (
      <>
        <div key={label} style={{ background: panel, border: `1px solid ${line}`, borderRadius: 22, padding: 30, minHeight: 190 }}>
          <div style={{ color, fontSize: 28, fontWeight: 900, marginBottom: 18 }}>{label}</div>
          <div style={{ color: '#dbeafe', fontSize: 25, lineHeight: 1.35 }}>{note}</div>
        </div>
        {index < 2 ? <div key={`${label}-arrow`} style={{ color: muted, fontSize: 38, fontWeight: 900 }}>→</div> : null}
      </>
    ))}
  </div>
);

const EvaluationTable = () => (
  <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 28 }}>
    <Card title="Correctness" body="Max error, mean error, and relative L2 error compare CPU and GPU fields." accent={green} />
    <Card title="Performance" body="CPU time, GPU kernel-only time, GPU end-to-end time, and speedup are reported separately." accent={warm} />
  </div>
);

const Cover: Page = () => (
  <div style={{ ...fill, padding: '146px 132px 96px' }}>
    <GridBackground />
    <div
      style={{
        position: 'absolute',
        right: 130,
        top: 160,
        width: 600,
        height: 600,
        borderRadius: 48,
        background:
          'radial-gradient(circle at 35% 30%, rgba(56,189,248,0.95), transparent 28%), radial-gradient(circle at 68% 65%, rgba(245,158,11,0.88), transparent 30%), linear-gradient(135deg, rgba(167,139,250,0.45), rgba(15,23,42,0.2))',
        filter: 'saturate(1.1)',
        opacity: 0.9,
      }}
    />
    <div style={{ position: 'relative', zIndex: 1, maxWidth: 1160 }}>
      <div style={{ color: 'var(--osd-accent)', fontSize: 36, fontWeight: 900, letterSpacing: '0.14em', textTransform: 'uppercase' }}>
        GPU Gems Chapter 38
      </div>
      <h1
        style={{
          fontFamily: 'var(--osd-font-display)',
          fontSize: 'var(--osd-size-hero)',
          lineHeight: 0.96,
          margin: '40px 0',
          letterSpacing: -4,
        }}
      >
        Fast Fluid Simulation on the GPU
      </h1>
      <p style={{ fontSize: 42, lineHeight: 1.35, color: muted, margin: 0, maxWidth: 980 }}>
        A CUDA implementation of a 2D Stable Fluids pipeline with CPU reference validation and real-time visualization.
      </p>
    </div>
  </div>
);

const Goal: Page = () => (
  <PageShell eyebrow="Project Goal" title="Reproduce the core simulation pipeline from Chapter 38">
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 28 }}>
      <Card title="Reference" body="Build a readable CPU implementation that acts as the numerical baseline." accent={green} />
      <Card title="Acceleration" body="Map the same per-cell operations to CUDA kernels for GPU execution." />
      <Card title="Validation" body="Compare CPU and GPU fields before presenting performance results." accent={warm} />
    </div>
  </PageShell>
);

const Ch38Pipeline: Page = () => (
  <PageShell eyebrow="Chapter 38" title="Stable Fluids decomposes Navier-Stokes into reusable operators">
    <div style={{ display: 'grid', gridTemplateColumns: '1.2fr 0.8fr', gap: 44, alignItems: 'center' }}>
      <FlowDiagram />
      <BulletList
        items={[
          'The simulation is grid-based and assumes incompressible flow.',
          'Each operator reads old fields and writes updated fields.',
          'Projection enforces the divergence-free condition.',
        ]}
      />
    </div>
  </PageShell>
);

const Framework: Page = () => (
  <PageShell eyebrow="Framework" title="Eulerian grid with semi-Lagrangian advection">
    <div style={{ display: 'grid', gridTemplateColumns: '0.95fr 1.05fr', gap: 36, alignItems: 'center' }}>
      <GridSketch />
      <div style={{ display: 'grid', gap: 22 }}>
        <Card title="Eulerian storage" body="Velocity, pressure, divergence, and dye live on fixed 2D grid cells." />
        <CodeBox>{'idx(x, y, width) = y * width + x'}</CodeBox>
      </div>
    </div>
  </PageShell>
);

const DataMapping: Page = () => (
  <PageShell eyebrow="Data Mapping" title="Chapter 38 uses textures; this project uses arrays and CUDA memory">
    <ComparisonTable />
  </PageShell>
);

const Fields: Page = () => (
  <PageShell eyebrow="Simulation State" title="The simulation stores vector, scalar, and visualization fields">
    <FieldMatrix />
  </PageShell>
);

const CpuGpu: Page = () => (
  <PageShell eyebrow="Execution Model" title="CPU and GPU run the same algorithm with different parallelism">
    <ParallelDiagram />
    <p style={{ fontSize: 36, lineHeight: 1.45, margin: '48px 0 0', color: muted }}>
      {/* @slide-comment id="c-3f84ea42" ts="2026-06-04T16:56:40.413Z" text="eyJub3RlIjoidXAgYSBsaXR0bGUgYml0In0" */}
      The timestep order is still sequential, but each kernel updates all interior cells in parallel.
    </p>
  </PageShell>
);

const Advection: Page = () => (
  <PageShell eyebrow="Advection" title="Velocity and dye are transported by the velocity field">
    <div style={{ display: 'grid', gridTemplateColumns: '1.1fr 0.9fr', gap: 44, alignItems: 'center' }}>
      <AdvectionVisual />
      <BulletList
        items={[
          'Velocity advection is self-advection.',
          'Dye advection moves a passive tracer.',
          'Bilinear sampling keeps motion smooth between grid cells.',
        ]}
      />
    </div>
  </PageShell>
);

const Diffusion: Page = () => (
  <PageShell eyebrow="Diffusion" title="Viscosity is solved with implicit Jacobi iteration">
    <div style={{ display: 'grid', gridTemplateColumns: '0.95fr 1.05fr', gap: 44, alignItems: 'center' }}>
      <BulletList
        items={[
          'Diffusion smooths velocity differences between neighboring cells.',
          'vx and vy are solved separately.',
          'The step is enabled only when viscosity is greater than zero.',
        ]}
      />
      <CodeBox>{'x_new(i,j) =\n  (x0(i,j) + a * neighbor_sum)\n  / (1 + 4a)'}</CodeBox>
    </div>
  </PageShell>
);

const ForceAndDye: Page = () => (
  <PageShell eyebrow="Forces and Dye" title="Forces affect velocity; dye only reveals the flow">
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 28 }}>
      <Card title="Gaussian force" body="A fixed center force gives the fluid a repeatable impulse for demos and tests." />
      <Card title="Mouse impulse" body="Dragging the mouse adds a local velocity impulse in the drag direction." accent={green} />
      <Card title="Passive dye" body="Dye concentration is advected by velocity but does not change pressure or density." accent={violet} />
    </div>
  </PageShell>
);

const Projection: Page = () => (
  <PageShell eyebrow="Projection" title="Projection makes the velocity field nearly incompressible">
    <div style={{ display: 'grid', gap: 38 }}>
      <ProjectionVisual />
      <BulletList
        items={[
          'Advection, diffusion, and forces can introduce divergence.',
          'Pressure is solved from the divergence field.',
          'Subtracting the pressure gradient reduces local expansion and compression.',
        ]}
      />
    </div>
  </PageShell>
);

const Divergence: Page = () => (
  <PageShell eyebrow="Divergence" title="Central differences measure local inflow and outflow">
    <CodeBox>{'div(i,j) = 0.5 / dx * (\n  vx(i+1,j) - vx(i-1,j)\n+ vy(i,j+1) - vy(i,j-1)\n)'}</CodeBox>
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr 1fr', gap: 24, marginTop: 42 }}>
      <Card title="Positive" body="Local flow tends to expand outward." accent={red} />
      <Card title="Near zero" body="Inflow and outflow are approximately balanced." accent={green} />
      <Card title="Negative" body="Local flow tends to compress inward." accent={violet} />
    </div>
  </PageShell>
);

const Visualization: Page = () => (
  <PageShell eyebrow="Visualization" title="OpenGL displays GPU fields in real time">
    <div style={{ display: 'grid', gridTemplateColumns: 'repeat(4, 1fr)', gap: 24 }}>
      <Card title="1" body="Velocity magnitude: speed only, not direction." accent={green} />
      <Card title="2" body="Pressure: scalar field from projection." accent={warm} />
      <Card title="3" body="Divergence: should be close to zero after projection." accent={red} />
      <Card title="4" body="Dye: passive tracer concentration." accent={violet} />
    </div>
    <p style={{ fontSize: 32, color: muted, margin: '46px 0 0', lineHeight: 1.45 }}>
      Velocity uses a heat map. Pressure and divergence use a diverging map: blue is negative, green is near zero, and warm colors are positive.
    </p>
  </PageShell>
);

const Command: Page = () => (
  <PageShell eyebrow="Demo Command" title="A typical GPU visualization run">
    <CodeBox>
      {'./build/fluid_sim visualize --backend gpu --width 512 --height 512 \\\n  --iters 40 --viscosity 0.00001 --diffusion-iters 20 \\\n  --dt 0.1 --steps-per-frame 1 --field dye --vsync 1'}
    </CodeBox>
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 24, marginTop: 38 }}>
      <Card title="Numerics" body="--iters controls pressure iterations; --diffusion-iters controls viscosity iterations." />
      <Card title="Timing" body="--dt is simulation time per step; --steps-per-frame controls how many steps run before rendering." accent={warm} />
    </div>
  </PageShell>
);

const Verification: Page = () => (
  <PageShell eyebrow="Evaluation" title="Correctness first, then performance">
    <EvaluationTable />
    <p style={{ fontSize: 36, lineHeight: 1.45, color: muted, marginTop: 52 }}>
      Performance comparison is meaningful only after CPU-GPU numerical agreement is verified.
    </p>
  </PageShell>
);

const Limits: Page = () => (
  <PageShell eyebrow="Scope" title="What this project does not simulate">
    <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 30 }}>
      <Card title="Not compressible" body="The solver assumes incompressible flow, so it does not model density waves or gas expansion." accent={red} />
      <Card title="Not full smoke physics" body="Dye is passive; there is no temperature, buoyancy, or density-force coupling." accent={violet} />
      <Card title="2D only" body="The implementation uses a 2D Cartesian grid rather than a 3D volume." />
      <Card title="Simple boundaries" body="The domain is a rectangular box with solid-wall boundary handling." accent={warm} />
    </div>
  </PageShell>
);

const Closing: Page = () => (
  <div style={{ ...fill, display: 'grid', placeItems: 'center', padding: 130, textAlign: 'center' }}>
    <GridBackground />
    <div style={{ position: 'relative', zIndex: 1, maxWidth: 1260 }}>
      <div style={{ color: 'var(--osd-accent)', fontSize: 52, fontWeight: 900, letterSpacing: '0.08em', textTransform: 'uppercase' }}>
        Conclusion
      </div>
      <h1 style={{ fontSize: 112, lineHeight: 1.04, margin: '36px 0', fontFamily: 'var(--osd-font-display)' }}>
        Same fluid algorithm, modern GPU execution.
      </h1>
      <p style={{ fontSize: 40, lineHeight: 1.45, color: muted, margin: 0 }}>
        The project replaces Chapter 38's texture and fragment shader implementation with CUDA kernels while preserving the Stable Fluids pipeline.
      </p>
    </div>
  </div>
);

export const meta: SlideMeta = {
  title: 'GPU Gems 38 Fluid Simulation',
  createdAt: '2026-06-04T16:06:53.133Z',
};

export default [
  Cover,
  Goal,
  Ch38Pipeline,
  Framework,
  DataMapping,
  Fields,
  CpuGpu,
  Advection,
  Diffusion,
  ForceAndDye,
  Projection,
  Divergence,
  Visualization,
  Command,
  Verification,
  Limits,
  Closing,
] satisfies Page[];
