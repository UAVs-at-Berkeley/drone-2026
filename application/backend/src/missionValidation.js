import yaml from "js-yaml";
import { z } from "zod";

const missionSchema = z.object({
  mission: z.object({
    steps: z.array(z.union([z.string(), z.record(z.any())])).min(1),
  }),
});

const filenameSchema = z
  .string()
  .trim()
  .min(1)
  .regex(/^[a-zA-Z0-9._-]+\.ya?ml$/, "Filename must end with .yaml or .yml.");

export function validateMissionYaml(yamlText) {
  let parsed;
  try {
    parsed = yaml.load(yamlText);
  } catch (error) {
    return { ok: false, message: `Invalid YAML syntax: ${error.message}` };
  }
  const result = missionSchema.safeParse(parsed);
  if (!result.success) {
    return { ok: false, message: "YAML must include mission.steps with at least one entry." };
  }
  return { ok: true, parsed };
}

export function validateMissionFilename(filename) {
  const result = filenameSchema.safeParse(filename);
  if (!result.success) {
    return { ok: false, message: result.error.issues[0]?.message || "Invalid filename." };
  }
  return { ok: true, filename: result.data };
}
