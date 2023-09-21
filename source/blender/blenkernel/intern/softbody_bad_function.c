#include "softbody_intern.h"

static CLG_LogRef LOG = {"bke.softbody"};

// int sb_detect_vertex_collisionCached(float opco[3],
//                                      float facenormal[3],
//                                      float *damp,
//                                      float force[3],
//                                      Object *vertexowner,
//                                      float time,
//                                      float vel[3],
//                                      float *intrusion)
// {
//   Object *ob = NULL;
//   GHash *hash;
//   GHashIterator *ihash;
//   float nv1[3], nv2[3], nv3[3], edge1[3], edge2[3], d_nvect[3], dv1[3], ve[3],
//       avel[3] = {0.0, 0.0, 0.0}, vv1[3], vv2[3], vv3[3], coledge[3] = {0.0f, 0.0f, 0.0f},
//       mindistedge = 1000.0f, outerforceaccu[3], innerforceaccu[3], facedist,
//       /* n_mag, */ /* UNUSED */ force_mag_norm, minx, miny, minz, maxx, maxy, maxz,
//       innerfacethickness = -0.5f, outerfacethickness = 0.2f, ee = 5.0f, ff = 0.1f, fa = 1;
//   int a, deflected = 0, cavel = 0, ci = 0;
//   /* init */
//   *intrusion = 0.0f;
//   hash = vertexowner->soft->scratch->colliderhash;
//   ihash = BLI_ghashIterator_new(hash);
//   outerforceaccu[0] = outerforceaccu[1] = outerforceaccu[2] = 0.0f;
//   innerforceaccu[0] = innerforceaccu[1] = innerforceaccu[2] = 0.0f;
//   /* go */
//   while (!BLI_ghashIterator_done(ihash)) {

//     ccd_Mesh *ccdm = BLI_ghashIterator_getValue(ihash);
//     ob = BLI_ghashIterator_getKey(ihash);
//     {
//       /* only with deflecting set */
//       if (ob->pd && ob->pd->deflect) {
//         const float(*vert_positions)[3] = NULL;
//         const float(*vert_positions_prev)[3] = NULL;
//         const MVertTri *vt = NULL;
//         const ccdf_minmax *mima = NULL;

//         if (ccdm) {
//           vert_positions = ccdm->vert_positions;
//           vert_positions_prev = ccdm->vert_positions_prev;
//           vt = ccdm->tri;
//           mima = ccdm->mima;
//           a = ccdm->tri_num;

//           minx = ccdm->bbmin[0];
//           miny = ccdm->bbmin[1];
//           minz = ccdm->bbmin[2];

//           maxx = ccdm->bbmax[0];
//           maxy = ccdm->bbmax[1];
//           maxz = ccdm->bbmax[2];

//           if ((opco[0] < minx) || (opco[1] < miny) || (opco[2] < minz) || (opco[0] > maxx) ||
//               (opco[1] > maxy) || (opco[2] > maxz))
//           {
//             /* Outside the padded bound-box -> collision object is too far away. */
//             BLI_ghashIterator_step(ihash);
//             continue;
//           }
//         }
//         else {
//           /* Aye that should be cached. */
//           CLOG_ERROR(&LOG, "missing cache error");
//           BLI_ghashIterator_step(ihash);
//           continue;
//         }

//         /* do object level stuff */
//         /* need to have user control for that since it depends on model scale */
//         innerfacethickness = -ob->pd->pdef_sbift;
//         outerfacethickness = ob->pd->pdef_sboft;
//         fa = (ff * outerfacethickness - outerfacethickness);
//         fa *= fa;
//         fa = 1.0f / fa;
//         avel[0] = avel[1] = avel[2] = 0.0f;
//         /* Use mesh. */
//         while (a--) {
//           if ((opco[0] < mima->minx) || (opco[0] > mima->maxx) || (opco[1] < mima->miny) ||
//               (opco[1] > mima->maxy) || (opco[2] < mima->minz) || (opco[2] > mima->maxz))
//           {
//             mima++;
//             vt++;
//             continue;
//           }

//           if (vert_positions) {

//             copy_v3_v3(nv1, vert_positions[vt->tri[0]]);
//             copy_v3_v3(nv2, vert_positions[vt->tri[1]]);
//             copy_v3_v3(nv3, vert_positions[vt->tri[2]]);

//             if (vert_positions_prev) {
//               /* Grab the average speed of the collider vertices before we spoil nvX
//                * humm could be done once a SB steps but then we' need to store that too
//                * since the AABB reduced probability to get here drastically
//                * it might be a nice tradeoff CPU <--> memory.
//                */
//               sub_v3_v3v3(vv1, nv1, vert_positions_prev[vt->tri[0]]);
//               sub_v3_v3v3(vv2, nv2, vert_positions_prev[vt->tri[1]]);
//               sub_v3_v3v3(vv3, nv3, vert_positions_prev[vt->tri[2]]);

//               mul_v3_fl(nv1, time);
//               madd_v3_v3fl(nv1, vert_positions_prev[vt->tri[0]], 1.0f - time);

//               mul_v3_fl(nv2, time);
//               madd_v3_v3fl(nv2, vert_positions_prev[vt->tri[1]], 1.0f - time);

//               mul_v3_fl(nv3, time);
//               madd_v3_v3fl(nv3, vert_positions_prev[vt->tri[2]], 1.0f - time);
//             }
//           }

//           /* Switch origin to be nv2. */
//           sub_v3_v3v3(edge1, nv1, nv2);
//           sub_v3_v3v3(edge2, nv3, nv2);
//           /* Abuse dv1 to have vertex in question at *origin* of triangle. */
//           sub_v3_v3v3(dv1, opco, nv2);

//           cross_v3_v3v3(d_nvect, edge2, edge1);
//           /* n_mag = */ /* UNUSED */ normalize_v3(d_nvect);
//           facedist = dot_v3v3(dv1, d_nvect);
//           /* so rules are */

//           if ((facedist > innerfacethickness) && (facedist < outerfacethickness)) {
//             if (isect_point_tri_prism_v3(opco, nv1, nv2, nv3)) {
//               force_mag_norm = (float)exp(-ee * facedist);
//               if (facedist > outerfacethickness * ff) {
//                 force_mag_norm = (float)force_mag_norm * fa * (facedist - outerfacethickness) *
//                                  (facedist - outerfacethickness);
//               }
//               *damp = ob->pd->pdef_sbdamp;
//               if (facedist > 0.0f) {
//                 *damp *= (1.0f - facedist / outerfacethickness);
//                 madd_v3_v3fl(outerforceaccu, d_nvect, force_mag_norm);
//                 deflected = 3;
//               }
//               else {
//                 madd_v3_v3fl(innerforceaccu, d_nvect, force_mag_norm);
//                 if (deflected < 2) {
//                   deflected = 2;
//                 }
//               }
//               if ((vert_positions_prev) && (*damp > 0.0f)) {
//                 choose_winner(ve, opco, nv1, nv2, nv3, vv1, vv2, vv3);
//                 add_v3_v3(avel, ve);
//                 cavel++;
//               }
//               *intrusion += facedist;
//               ci++;
//             }
//           }

//           mima++;
//           vt++;
//         } /* while a */
//       }   /* if (ob->pd && ob->pd->deflect) */
//       BLI_ghashIterator_step(ihash);
//     }
//   } /* while () */

//   if (deflected == 1) { /* no face but 'outer' edge cylinder sees vert */
//     force_mag_norm = (float)exp(-ee * mindistedge);
//     if (mindistedge > outerfacethickness * ff) {
//       force_mag_norm = (float)force_mag_norm * fa * (mindistedge - outerfacethickness) *
//                        (mindistedge - outerfacethickness);
//     }
//     madd_v3_v3fl(force, coledge, force_mag_norm);
//     *damp = ob->pd->pdef_sbdamp;
//     if (mindistedge > 0.0f) {
//       *damp *= (1.0f - mindistedge / outerfacethickness);
//     }
//   }
//   if (deflected == 2) { /* face inner detected */
//     add_v3_v3(force, innerforceaccu);
//   }
//   if (deflected == 3) { /* face outer detected */
//     add_v3_v3(force, outerforceaccu);
//   }

//   BLI_ghashIterator_free(ihash);
//   if (cavel) {
//     mul_v3_fl(avel, 1.0f / (float)cavel);
//   }
//   copy_v3_v3(vel, avel);
//   if (ci) {
//     *intrusion /= ci;
//   }
//   if (deflected) {
//     normalize_v3_v3(facenormal, force);
//   }
//   return deflected;
// }
