/*
 * MassMatrix.cpp
 *
 *  Created on: 19 jun 2021
 *     Authors: Oliva Alexander, Gaz Claudio, Cognetti Marco
 *
 *  C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, 'Dynamic
 *  Identification of the Franka Emika Panda Robot With Retrieval of Feasible
 *  Parameters Using Penalty-Based Optimization'. IEEE RA-L, 2019.
 *
 *  ver: 2.0
 *   This version is parametrized w.r.t. the payload parameters
 *   (mass mL, Center-of-Mass cl and inertia tensor I_L).
 *    - fMcom is the Homogeneous matrix of the CoM in flange frame.
 *    - mL is the total mass of the payload.   [kg]
 *    - I_L is the inertia tensor of the payload in the frame given by fMcom
 *
 */

#include "franka_model.h"

namespace franka_model
{

vpMatrix
massMatrix( const vpColVector &q, const double mL, const vpHomogeneousMatrix &fMcom, const vpMatrix &I_L )
{
  vpMatrix B( njoints, njoints ), J( 3, 3 );
  double cq1, cq2, cq3, cq4, cq5, cq6, sq1, sq2, sq3, sq4, sq5, sq6, pcq1, pcq2, pcq3, pcq4, pcq5, pcq6, psq1, psq2,
      psq4, psq5, psq6, cq13, cq14, cq15, cq16, cq23, cq24, cq25, cq26, cq34, cq35, cq36, cq45;
  double clx, cly, clz, XXL, XYL, XZL, YYL, YZL, ZZL;
  double sq12, sq14, sq15, sq16, sq23, sq24, sq25, sq26, sq34, sq35, sq36, sq45, sq46, sq56;

  clx = fMcom[0][3];
  cly = fMcom[1][3];
  clz = 0.107 + fMcom[2][3];

  vpColVector cl( { clx, cly, clz } );

  // Using the Steiner theorem to express the inertia tensor in link 7 frame
  J = fMcom.getRotationMatrix() * I_L * fMcom.getRotationMatrix().t() +
      mL * vpColVector::skew( cl ).t() * vpColVector::skew( cl );

  XXL = J[0][0];
  XYL = J[0][1];
  XZL = J[0][2];
  YYL = J[1][1];
  YZL = J[1][2];
  ZZL = J[2][2];

  cq1 = cos( q[1] );
  cq2 = cos( q[2] );
  cq3 = cos( q[3] );
  cq4 = cos( q[4] );
  cq5 = cos( q[5] );
  cq6 = cos( q[6] );

  cq13 = cq1 * cq3;
  cq14 = cq1 * cq4;
  cq15 = cq1 * cq5;
  cq16 = cq1 * cq6;
  cq23 = cq2 * cq3;
  cq24 = cq2 * cq4;
  cq25 = cq2 * cq5;
  cq26 = cq2 * cq6;
  cq34 = cq3 * cq4;
  cq35 = cq3 * cq5;
  cq36 = cq3 * cq6;
  cq45 = cq4 * cq5;

  sq1 = sin( q[1] );
  sq2 = sin( q[2] );
  sq3 = sin( q[3] );
  sq4 = sin( q[4] );
  sq5 = sin( q[5] );
  sq6 = sin( q[6] );

  sq12 = sq1 * sq2;
  sq14 = sq1 * sq4;
  sq15 = sq1 * sq5;
  sq16 = sq1 * sq6;
  sq23 = sq2 * sq3;
  sq24 = sq2 * sq4;
  sq25 = sq2 * sq5;
  sq26 = sq2 * sq6;
  sq34 = sq3 * sq4;
  sq35 = sq3 * sq5;
  sq36 = sq3 * sq6;
  sq45 = sq4 * sq5;
  sq46 = sq4 * sq6;
  sq56 = sq5 * sq6;

  pcq1 = cq1 * cq1;
  pcq2 = cq2 * cq2;
  pcq3 = cq3 * cq3;
  pcq4 = cq4 * cq4;
  pcq5 = cq5 * cq5;
  pcq6 = cq6 * cq6;

  psq1 = sq1 * sq1;
  psq2 = sq2 * sq2;
  psq4 = sq4 * sq4;
  psq5 = sq5 * sq5;
  psq6 = sq6 * sq6;

  B[0][0] =
      pow( cq1 * 0.0825 + cq2 * sq1 * 0.316, 2.0 ) * 3.587895 -
      ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( sq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                    sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) ) -
            cq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) *
          3.505431787E-2 +
      ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                    sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) ) +
            sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) *
          3.505431787E-2 -
      cq1 * sq1 * 7.85E-3 -
      ( ( sq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) +
          cq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
            ( cq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                      cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                      sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                      cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                              cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) -
              sq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                              sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                      cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                      sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) ) *
            2.0 -
        ( cq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) -
          sq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
            ( sq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                      cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                      sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                      cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                              cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
              cq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                              sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                      cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                      sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) ) *
            2.0 ) *
          ( clz * mL + 4.5305948634E-2 ) +
      ( mL + 7.35522E-1 ) *
          pow( sq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                       cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                       sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                       cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                               cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                   cq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                                   sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                           cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                           sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ),
               2.0 ) +
      ( mL + 7.35522E-1 ) *
          pow( cq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                       cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                       sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                       cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                               cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) -
                   sq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                                   sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                           cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                           sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ),
               2.0 ) +
      pcq1 * 2.811E-2 +
      ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( ( sq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) +
              cq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XZL - 1.196E-3 ) +
            ( YZL - 7.41E-4 ) * ( cq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) -
                                  sq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) +
                                          sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) +
            ( ZZL + 4.815E-3 ) *
                ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) -
      psq1 * 1.27733849867456E-1 -
      ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) *
          ( cq13 * 4.037E-3 - cq4 * ( cq1 * sq3 - cq23 * sq1 ) * 3.5549E-2 -
            sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 2.117E-3 - sq12 * sq4 * 3.5549E-2 + cq2 * sq1 * sq3 * 4.037E-3 +
            cq4 * sq12 * 2.117E-3 ) -
      ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) *
          ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 1.158E-3 +
            sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 3.41E-4 -
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 3.41E-4 - sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 5.433E-3 +
            sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.158E-3 + cq4 * sq12 * 5.433E-3 ) +
      ( cly * mL - 3.127439544E-3 ) *
          ( ( sq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) +
              cq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( sq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                          sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) -
                  sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 0.088 -
                  cq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) + cq4 * sq12 * 0.088 ) *
                2.0 -
            ( sq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                      cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                      sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                      cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                              cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
              cq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                              sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                      cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                      sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) ) *
                ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) * 2.0 ) -
      ( cq13 + cq2 * sq1 * sq3 ) * ( cq1 * 0.0825 + cq2 * sq1 * 0.316 ) * 3.815367543E-1 -
      ( cq1 * sq3 - cq23 * sq1 ) * ( cq1 * 0.0825 + cq2 * sq1 * 0.316 ) * 7.4928881601E-1 +
      pow( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
               sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                       cq2 * sq1 * sq3 * 0.0825 ),
           2.0 ) *
          1.225946 +
      pow( sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
               cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                       cq2 * sq1 * sq3 * 0.0825 ),
           2.0 ) *
          2.892501 -
      ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) *
          ( sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
            cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) ) *
          9.4243372804E-2 +
      ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) *
          ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
            sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) ) *
          9.4243372804E-2 -
      ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 1.09E-4 -
            sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 4.354E-3 +
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 4.354E-3 + sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 3.41E-4 +
            sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.09E-4 - cq4 * sq12 * 3.41E-4 ) -
      ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) *
          ( cq13 * 2.29E-4 - cq4 * ( cq1 * sq3 - cq23 * sq1 ) * 2.117E-3 -
            sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 2.9474E-2 - sq12 * sq4 * 2.117E-3 + cq2 * sq1 * sq3 * 2.29E-4 +
            cq4 * sq12 * 2.9474E-2 ) +
      cq1 * ( cq1 * 1.083E-2 + cq2 * sq1 * 1.1396E-2 - sq12 * 1.2805E-2 ) +
      ( cq13 + cq2 * sq1 * sq3 ) * ( cq13 * 1.9552E-2 + cq1 * sq3 * 7.796E-3 - sq12 * 8.640999999999999E-3 -
                                     cq23 * sq1 * 7.796E-3 + cq2 * sq1 * sq3 * 1.9552E-2 ) -
      ( sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
        cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) ) *
          ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          4.705351387E-2 +
      ( sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
        cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) ) *
          ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          2.0048323339E-1 +
      ( cq13 + cq2 * sq1 * sq3 ) * ( cq13 * 8.626999999999999E-3 - cq4 * ( cq1 * sq3 - cq23 * sq1 ) * 4.037E-3 -
                                     sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 2.29E-4 - sq12 * sq4 * 4.037E-3 +
                                     cq2 * sq1 * sq3 * 8.626999999999999E-3 + cq4 * sq12 * 2.29E-4 ) +
      ( cq1 * sq3 - cq23 * sq1 ) * ( cq13 * 7.796E-3 + cq1 * sq3 * 2.5853E-2 + sq12 * 1.332E-3 -
                                     cq23 * sq1 * 2.5853E-2 + cq2 * sq1 * sq3 * 7.796E-3 ) +
      pow( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                   sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                           cq2 * sq1 * sq3 * 0.0825 ) ) +
               sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ),
           2.0 ) *
          1.666555 +
      pow( sq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                   sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                           cq2 * sq1 * sq3 * 0.0825 ) ) -
               cq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ),
           2.0 ) *
          1.666555 +
      ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) *
          ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                    sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) ) +
            sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) *
          4.705351387E-2 -
      ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) *
          ( sq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                    sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) ) -
            cq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) *
          2.0048323339E-1 +
      pcq2 * psq1 * 3.22395481024E-1 -
      ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
        sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) ) *
          ( cq13 + cq2 * sq1 * sq3 ) * 1.0068694498E-1 +
      ( sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
        cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) ) *
          ( cq13 + cq2 * sq1 * sq3 ) * 2.9307465076E-2 +
      ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 1.964E-3 -
            sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 1.09E-4 +
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.09E-4 - sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 1.158E-3 +
            sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.964E-3 + cq4 * sq12 * 1.158E-3 ) +
      ( cq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) -
        sq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          ( ( cq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) -
              sq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( YYL + 1.0027E-2 ) +
            ( YZL - 7.41E-4 ) *
                ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) +
            ( sq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) +
              cq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XYL - 4.28E-4 ) ) +
      pow( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ), 2.0 ) * 1.225946 + psq1 * psq2 * 3.22395481024E-1 +
      ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) *
          1.0068694498E-1 -
      ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) *
          2.9307465076E-2 -
      ( clx * mL + 7.735484874E-3 ) *
          ( ( cq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) -
              sq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( sq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                          sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) -
                  sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 0.088 -
                  cq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) + cq4 * sq12 * 0.088 ) *
                2.0 -
            ( cq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                      cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                      sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                      cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                              cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) -
              sq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                              sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                      cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                      sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) ) *
                ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) * 2.0 ) +
      ( sq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) +
        cq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          ( ( sq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) +
              cq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XXL + 1.2516E-2 ) +
            ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
                ( XZL - 1.196E-3 ) +
            ( cq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) -
              sq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XYL - 4.28E-4 ) ) +
      ( mL + 7.35522E-1 ) *
          pow( sq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                       sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                               cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) -
                   sq4 * ( cq1 * sq3 - cq23 * sq1 ) * 0.088 -
                   cq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) + cq4 * sq12 * 0.088,
               2.0 ) +
      sq12 * ( cq1 * ( -1.2805E-2 ) + cq2 * sq1 * 4.761E-3 + sq12 * 3.6155E-2 ) +
      psq1 * psq2 * pow( cq3 * 0.316 - sq3 * 0.0825, 2.0 ) * 3.587895 +
      psq1 * psq2 * pow( cq3 * 0.0825 + sq3 * 0.316, 2.0 ) * 3.587895 -
      sq12 * ( cq13 * 8.640999999999999E-3 - cq1 * sq3 * 1.332E-3 - sq12 * 2.8323E-2 + cq23 * sq1 * 1.332E-3 +
               cq2 * sq1 * sq3 * 8.640999999999999E-3 ) +
      psq1 * psq2 * ( cq3 * 0.316 - sq3 * 0.0825 ) * 7.4928881601E-1 -
      psq1 * psq2 * ( cq3 * 0.0825 + sq3 * 0.316 ) * 3.815367543E-1 +
      cq2 * sq1 * ( cq1 * 1.1396E-2 + cq2 * sq1 * 3.7242E-2 + sq12 * 4.761E-3 ) + cq1 * cq2 * sq1 * 5.6149866119104E-2 -
      cq1 * sq12 * 8.0092831779456E-2 +
      sq12 * ( cq13 + cq2 * sq1 * sq3 ) * ( cq3 * 0.316 - sq3 * 0.0825 ) * 1.9700413866E-1 +
      sq12 * ( cq1 * sq3 - cq23 * sq1 ) * ( cq3 * 0.0825 + sq3 * 0.316 ) * 1.9700413866E-1 + 9.117E-3;
  B[0][1] =
      cq1 * ( -7.04E-4 ) - sq1 * 2.127579280274999E-3 -
      ( clz * mL + 4.5305948634E-2 ) *
          ( ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( sq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                          cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                          cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
                  cq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                          cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                                  cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                          sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) +
            ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( cq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                          cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                          cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                  sq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                          cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                                  cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                          sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) +
            ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
              sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                          cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                          cq34 * sq25 * 0.088 ) +
                  cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                  sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                          cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) +
            ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
              cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                  sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                          cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
                  cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                          cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                          cq34 * sq25 * 0.088 ) ) ) +
      ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq13 + cq2 * sq1 * sq3 ) * 1.4653732538E-2 -
      ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq13 + cq2 * sq1 * sq3 ) * 5.034347249E-2 -
      cq1 * cq2 * 8.919761981448002E-3 -
      ( cq2 * sq4 + cq34 * sq2 ) *
          ( cq13 * 4.037E-3 - sq12 * sq4 * 3.5549E-2 - cq14 * sq3 * 3.5549E-2 + cq2 * sq1 * sq3 * 4.037E-3 +
            cq4 * sq12 * 2.117E-3 - cq1 * sq34 * 2.117E-3 + cq23 * cq4 * sq1 * 3.5549E-2 + cq23 * sq14 * 2.117E-3 ) *
          0.5 -
      ( cly * mL - 3.127439544E-3 ) *
          ( -( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
                  cq4 * sq12 * 0.088 - cq1 * sq34 * 0.088 + cq23 * sq14 * 0.088 +
                  cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) +
            ( sq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                      cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                      cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                              cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                      sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
              cq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                      cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                              cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                      sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
                ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) +
            ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
                ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                          cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                          cq34 * sq25 * 0.088 ) +
                  cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                  sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                          cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) +
            ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
              cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( cq24 * 0.088 +
                  sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                          sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
                  cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) ) -
      sq2 * ( cq1 * 1.1396E-2 + cq2 * sq1 * 3.7242E-2 + sq12 * 4.761E-3 ) * 0.5 +
      ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) *
          ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.964E-3 -
            sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.09E-4 +
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.09E-4 + sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.964E-3 +
            cq4 * sq12 * 1.158E-3 - cq1 * sq34 * 1.158E-3 + cq23 * sq14 * 1.158E-3 ) *
          0.5 -
      cq1 * sq2 * 2.8074933059552E-2 - cq4 * sq1 * 1.4459906121375E-3 -
      ( cq13 + cq2 * sq1 * sq3 ) * ( cq2 * 8.640999999999999E-3 - cq3 * sq2 * 7.796E-3 + sq23 * 1.9552E-2 ) * 0.5 +
      cq2 * ( cq1 * ( -1.2805E-2 ) + cq2 * sq1 * 4.761E-3 + sq12 * 3.6155E-2 ) * 0.5 +
      ( sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
        cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) * 1.00241616695E-1 -
      ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
        cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) * 2.3526756935E-2 +
      ( cq24 - cq3 * sq24 ) *
          ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.158E-3 +
            sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 3.41E-4 -
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 3.41E-4 + sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.158E-3 +
            cq4 * sq12 * 5.433E-3 - cq1 * sq34 * 5.433E-3 + cq23 * sq14 * 5.433E-3 ) *
          0.5 -
      ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq24 * ( -1.158E-3 ) + sq23 * sq5 * 1.964E-3 - cq25 * sq4 * 1.964E-3 + cq3 * sq24 * 1.158E-3 +
            cq5 * sq23 * 1.09E-4 + cq2 * sq45 * 1.09E-4 - cq34 * cq5 * sq2 * 1.964E-3 + cq34 * sq25 * 1.09E-4 ) *
          0.5 +
      ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) *
          ( cq24 * ( -2.117E-3 ) + cq2 * sq4 * 3.5549E-2 + sq23 * 4.037E-3 + cq34 * sq2 * 3.5549E-2 +
            cq3 * sq24 * 2.117E-3 ) *
          0.5 +
      ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) +
        cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) * 1.225946 -
            sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) * 1.225946 ) -
      ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) -
        sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) * 1.225946 +
            cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) * 1.225946 ) +
      ( cq24 - cq3 * sq24 ) *
          ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
            cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          1.00241616695E-1 -
      ( cq24 - cq3 * sq24 ) *
          ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
            sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          2.3526756935E-2 -
      ( cq13 + cq2 * sq1 * sq3 ) *
          ( cq24 * ( -2.29E-4 ) + cq2 * sq4 * 4.037E-3 + sq23 * 8.626999999999999E-3 + cq34 * sq2 * 4.037E-3 +
            cq3 * sq24 * 2.29E-4 ) *
          0.5 -
      cq2 *
          ( cq13 * 8.640999999999999E-3 - cq1 * sq3 * 1.332E-3 - sq12 * 2.8323E-2 + cq23 * sq1 * 1.332E-3 +
            cq2 * sq1 * sq3 * 8.640999999999999E-3 ) *
          0.5 -
      pcq2 * sq1 * 1.625284143945E-2 -
      ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
        sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          ( ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XYL - 4.28E-4 ) -
            ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( YYL + 1.0027E-2 ) +
            ( YZL - 7.41E-4 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) ) *
          0.5 -
      sq2 *
          ( cq1 * ( -0.0825 ) - cq13 * 0.0825 + cq2 * sq1 * 0.768 + cq1 * sq3 * 0.316 + cq1 * pcq3 * 0.165 -
            cq2 * pcq3 * sq1 * 0.768 + cq13 * sq3 * 0.768 - cq2 * sq1 * sq3 * 0.165 + cq23 * sq1 * sq3 * 0.165 ) *
          4.7121686402E-2 +
      ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq24 * 3.41E-4 + sq23 * sq5 * 1.09E-4 - cq25 * sq4 * 1.09E-4 - cq3 * sq24 * 3.41E-4 +
            cq5 * sq23 * 4.354E-3 + cq2 * sq45 * 4.354E-3 - cq34 * cq5 * sq2 * 1.09E-4 + cq34 * sq25 * 4.354E-3 ) *
          0.5 +
      ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) *
          ( cq24 * 5.433E-3 - sq23 * sq5 * 1.158E-3 + cq25 * sq4 * 1.158E-3 - cq3 * sq24 * 5.433E-3 +
            cq5 * sq23 * 3.41E-4 + cq2 * sq45 * 3.41E-4 + cq34 * cq5 * sq2 * 1.158E-3 + cq34 * sq25 * 3.41E-4 ) *
          0.5 -
      ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
        cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          ( ( XZL - 1.196E-3 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
            ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XYL - 4.28E-4 ) +
            ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XXL + 1.2516E-2 ) ) *
          0.5 -
      ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) *
          ( cq24 * ( -2.9474E-2 ) + cq2 * sq4 * 2.117E-3 + sq23 * 2.29E-4 + cq34 * sq2 * 2.117E-3 +
            cq3 * sq24 * 2.9474E-2 ) *
          0.5 +
      ( clx * mL + 7.735484874E-3 ) *
          ( ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
                  cq4 * sq12 * 0.088 - cq1 * sq34 * 0.088 + cq23 * sq14 * 0.088 +
                  cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) +
            ( cq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                      cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                      cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                              cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                      sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
              sq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                      cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                              cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                      sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
                ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
            ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
                ( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                  sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                          cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
                  cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                          cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                          cq34 * sq25 * 0.088 ) ) +
            ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
              sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( cq24 * 0.088 +
                  sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                          sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
                  cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) ) -
      ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) *
          ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.09E-4 -
            sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 4.354E-3 +
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 4.354E-3 + sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.09E-4 -
            cq4 * sq12 * 3.41E-4 + cq1 * sq34 * 3.41E-4 - cq23 * sq14 * 3.41E-4 ) *
          0.5 +
      ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) -
        sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) * 2.3526756935E-2 -
      cq1 * ( cq2 * 1.2805E-2 + sq2 * 1.1396E-2 ) * 0.5 +
      ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
          ( ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
              cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XYL - 4.28E-4 ) +
            ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
              sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( YYL + 1.0027E-2 ) -
            ( YZL - 7.41E-4 ) *
                ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          0.5 +
      ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XZL - 1.196E-3 ) +
            ( ZZL + 4.815E-3 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
            ( YZL - 7.41E-4 ) *
                ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) ) *
          0.5 -
      ( ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
          cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
            ( XZL - 1.196E-3 ) -
        ( ZZL + 4.815E-3 ) *
            ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) +
        ( YZL - 7.41E-4 ) *
            ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
              sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) ) *
          ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) * 0.5 +
      sq2 * ( cq13 * 0.316 + cq1 * sq3 * 0.0825 - cq23 * sq1 * 0.165 ) * 1.9076837715E-1 -
      ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) -
        sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) * 1.00241616695E-1 -
      ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
          ( ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
              sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XYL - 4.28E-4 ) -
            ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
                ( XZL - 1.196E-3 ) +
            ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
              cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XXL + 1.2516E-2 ) ) *
          0.5 -
      sq2 * ( cq13 * 0.0825 - cq1 * sq3 * 0.316 + cq2 * sq1 * sq3 * 0.165 ) * 3.74644408005E-1 +
      ( cq1 * sq3 - cq23 * sq1 ) * ( cq2 * 1.332E-3 + cq3 * sq2 * 2.5853E-2 - sq23 * 7.796E-3 ) * 0.5 +
      ( cq24 - cq3 * sq24 ) *
          ( cq13 * 2.29E-4 - sq12 * sq4 * 2.117E-3 - cq14 * sq3 * 2.117E-3 + cq2 * sq1 * sq3 * 2.29E-4 +
            cq4 * sq12 * 2.9474E-2 - cq1 * sq34 * 2.9474E-2 + cq23 * cq4 * sq1 * 2.117E-3 + cq23 * sq14 * 2.9474E-2 ) *
          0.5 +
      ( sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
        cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( sq5 *
                ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                          cq2 * sq1 * sq3 * 0.0825 ) +
                  cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
                1.666555 +
            cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 1.666555 ) +
      ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                        cq2 * sq1 * sq3 * 0.0825 ) +
                cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
        sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
                1.666555 -
            cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 1.666555 ) -
      sq2 * ( cq1 * 0.0825 + cq2 * sq1 * 0.316 ) * 1.13377482 -
      ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) -
        sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) * 1.666555 +
            cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) * 1.666555 ) -
      ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          2.3526756935E-2 +
      ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          1.00241616695E-1 -
      sq23 *
          ( cq13 * 1.9552E-2 + cq1 * sq3 * 7.796E-3 - sq12 * 8.640999999999999E-3 - cq23 * sq1 * 7.796E-3 +
            cq2 * sq1 * sq3 * 1.9552E-2 ) *
          0.5 +
      sq12 * ( cq2 * 3.6155E-2 - sq2 * 4.761E-3 ) * 0.5 +
      cq3 * sq2 *
          ( cq13 * 7.796E-3 + cq1 * sq3 * 2.5853E-2 + sq12 * 1.332E-3 - cq23 * sq1 * 2.5853E-2 +
            cq2 * sq1 * sq3 * 7.796E-3 ) *
          0.5 -
      cq2 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 5.034347249E-2 -
      cq2 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 1.4653732538E-2 -
      sq23 *
          ( cq13 * 8.626999999999999E-3 - sq12 * sq4 * 4.037E-3 - cq14 * sq3 * 4.037E-3 +
            cq2 * sq1 * sq3 * 8.626999999999999E-3 + cq4 * sq12 * 2.29E-4 - cq1 * sq34 * 2.29E-4 +
            cq23 * cq4 * sq1 * 4.037E-3 + cq23 * sq14 * 2.29E-4 ) *
          0.5 +
      pcq2 * cq4 * sq1 * 2.891981224275E-3 + cq2 * sq1 * ( cq2 * 4.761E-3 - sq2 * 3.7242E-2 ) * 0.5 +
      sq23 *
          ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) +
            cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          5.034347249E-2 +
      sq23 *
          ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) -
            sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          1.4653732538E-2 +
      ( mL + 7.35522E-1 ) *
          ( cq24 * 0.088 +
            sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
            cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) *
          ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
            cq4 * sq12 * 0.088 - cq1 * sq34 * 0.088 + cq23 * sq14 * 0.088 +
            cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) +
      ( mL + 7.35522E-1 ) *
          ( sq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                    cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                    cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
            cq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                    cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                            cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                    sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
          ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                    cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                    cq34 * sq25 * 0.088 ) +
            cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                            sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                    cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) -
      ( mL + 7.35522E-1 ) *
          ( cq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                    cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                    cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
            sq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                    cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                            cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                    sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
          ( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                            sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                    cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
            cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                    cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                    cq34 * sq25 * 0.088 ) ) -
      cq1 * cq24 * 5.538582223460001E-3 +
      sq12 * ( cq2 * 2.8323E-2 + cq3 * sq2 * 1.332E-3 + sq23 * 8.640999999999999E-3 ) * 0.5 +
      cq34 * sq1 * 1.4459906121375E-3 - cq1 * sq24 * 6.730429031040001E-3 + cq4 * sq1 * sq3 * 6.730429031040001E-3 -
      pcq2 * cq34 * sq1 * 2.891981224275E-3 + cq1 * pcq3 * sq24 * 1.346085806208E-2 -
      pcq2 * cq4 * sq1 * sq3 * 1.346085806208E-2 + cq2 * sq12 * pow( cq3 * 0.316 - sq3 * 0.0825, 2.0 ) * 3.587895 +
      cq2 * sq12 * pow( cq3 * 0.0825 + sq3 * 0.316, 2.0 ) * 3.587895 +
      cq2 * sq12 * pow( cq3 * 0.0825 + sq3 * 0.316 - 0.0825, 2.0 ) * 1.225946 -
      sq12 * ( cq2 * sq4 + cq34 * sq2 ) * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 5.034347249E-2 -
      sq12 * ( cq24 - cq3 * sq24 ) * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 1.4653732538E-2 -
      cq1 * cq23 * cq4 * 6.730429031040001E-3 + cq1 * cq24 * sq3 * 1.4459906121375E-3 +
      cq13 * sq24 * 5.538582223460001E-3 + cq1 * sq23 * sq4 * 1.4459906121375E-3 +
      cq2 * pcq3 * sq12 * sq4 * 2.891981224275E-3 - cq23 * sq12 * sq4 * 2.891981224275E-3 -
      cq13 * sq23 * sq4 * 2.891981224275E-3 + cq23 * sq12 * sq34 * 1.346085806208E-2;
  B[0][2] =
      cq1 * ( -9.393448129859976E-4 ) - cq13 * 3.147678222975E-2 + cq2 * sq1 * 1.29119816581387E-1 -
      cq1 * sq3 * 5.4041249064495E-2 +
      ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) *
          ( cq35 * ( -3.41E-4 ) + cq3 * sq5 * 1.158E-3 - sq34 * 5.433E-3 + cq45 * sq3 * 1.158E-3 +
            cq4 * sq35 * 3.41E-4 ) *
          0.5 +
      cq3 *
          ( cq13 * 1.9552E-2 + cq1 * sq3 * 7.796E-3 - sq12 * 8.640999999999999E-3 - cq23 * sq1 * 7.796E-3 +
            cq2 * sq1 * sq3 * 1.9552E-2 ) *
          0.5 -
      ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) * ( cq3 * ( -2.29E-4 ) + cq4 * sq3 * 2.117E-3 + sq34 * 2.9474E-2 ) *
          0.5 -
      sq12 * 2.1724761981448E-2 -
      ( clx * mL + 7.735484874E-3 ) *
          ( ( cq35 - cq4 * sq35 ) *
                ( cq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                          cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                          cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                  sq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                          cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                                  cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                          sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) -
            ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
                ( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
                  cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) -
            ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) *
                ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
                  cq4 * sq12 * 0.088 - cq1 * sq34 * 0.088 + cq23 * sq14 * 0.088 +
                  cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) +
            sq4 *
                ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
                  sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( sq3 * 0.088 - sq5 * 0.0825 + cq3 * sq5 * 0.0825 + sq35 * 0.384 ) ) +
      ( clz * mL + 4.5305948634E-2 ) *
          ( cq15 * ( -0.768 ) + cq1 * pcq3 * cq5 * 0.768 - cq24 * sq1 * 0.088 - cq25 * sq1 * 0.0825 +
            cq15 * sq3 * 0.165 + cq2 * pcq3 * cq4 * sq1 * 0.176 + cq2 * pcq3 * cq5 * sq1 * 0.165 +
            cq24 * pcq5 * sq1 * 0.176 - cq1 * pcq3 * cq4 * sq5 * 0.165 + cq1 * pcq3 * cq5 * sq5 * 0.176 -
            cq1 * pcq4 * cq5 * sq5 * 0.176 - cq13 * cq4 * sq3 * 0.176 - cq23 * cq5 * sq1 * 0.0825 -
            cq13 * cq5 * sq3 * 0.165 + cq13 * cq4 * sq5 * 0.165 + cq25 * sq1 * sq3 * 0.316 - cq24 * sq15 * 0.384 -
            cq3 * sq12 * sq4 * 0.088 - sq12 * sq45 * 0.316 + cq2 * pcq3 * cq4 * sq15 * 0.768 +
            cq3 * pcq5 * sq12 * sq4 * 0.176 - cq2 * pcq3 * cq4 * pcq5 * sq1 * 0.352 +
            cq1 * pcq3 * pcq4 * cq5 * sq5 * 0.176 + cq23 * cq5 * sq1 * sq3 * 0.768 + cq23 * cq4 * sq15 * 0.316 -
            cq13 * cq4 * sq35 * 0.768 + cq24 * sq1 * sq35 * 0.0825 - cq3 * sq12 * sq45 * 0.384 +
            sq12 * sq34 * sq5 * 0.0825 + cq13 * cq4 * pcq5 * sq3 * 0.352 - cq23 * cq4 * sq1 * sq35 * 0.165 +
            cq23 * cq5 * sq1 * sq35 * 0.176 - cq45 * sq12 * sq34 * sq5 * 0.176 +
            cq23 * pcq4 * cq5 * sq1 * sq35 * 0.176 ) +
      sq3 *
          ( cq13 * 7.796E-3 + cq1 * sq3 * 2.5853E-2 + sq12 * 1.332E-3 - cq23 * sq1 * 2.5853E-2 +
            cq2 * sq1 * sq3 * 7.796E-3 ) *
          0.5 +
      ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
        sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          ( ( YYL + 1.0027E-2 ) * ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) +
            ( YZL - 7.41E-4 ) * ( cq35 - cq4 * sq35 ) -
            ( XYL - 4.28E-4 ) * ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) ) *
          0.5 +
      cq3 *
          ( cq13 * 8.626999999999999E-3 - sq12 * sq4 * 4.037E-3 - cq14 * sq3 * 4.037E-3 +
            cq2 * sq1 * sq3 * 8.626999999999999E-3 + cq4 * sq12 * 2.29E-4 - cq1 * sq34 * 2.29E-4 +
            cq23 * cq4 * sq1 * 4.037E-3 + cq23 * sq14 * 2.29E-4 ) *
          0.5 +
      ( cq3 * sq5 + cq45 * sq3 ) *
          ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.964E-3 -
            sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.09E-4 +
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.09E-4 + sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.964E-3 +
            cq4 * sq12 * 1.158E-3 - cq1 * sq34 * 1.158E-3 + cq23 * sq14 * 1.158E-3 ) *
          0.5 -
      cq3 *
          ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) +
            cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          5.034347249E-2 -
      cq3 *
          ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) -
            sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          1.4653732538E-2 +
      ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
        cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          ( ( XYL - 4.28E-4 ) * ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) -
            ( XXL + 1.2516E-2 ) * ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) +
            ( cq35 - cq4 * sq35 ) * ( XZL - 1.196E-3 ) ) *
          0.5 +
      cq1 * pcq3 * 3.6189455156736E-2 +
      ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) *
          ( cq3 * ( -4.037E-3 ) + cq4 * sq3 * 3.5549E-2 + sq34 * 2.117E-3 ) * 0.5 +
      ( cq13 + cq2 * sq1 * sq3 ) * ( cq3 * 1.9552E-2 + sq3 * 7.796E-3 ) * 0.5 +
      ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) *
          ( ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
              cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XYL - 4.28E-4 ) +
            ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
              sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( YYL + 1.0027E-2 ) -
            ( YZL - 7.41E-4 ) *
                ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          0.5 -
      ( ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
          sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
            ( XYL - 4.28E-4 ) -
        ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
            ( XZL - 1.196E-3 ) +
        ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
          cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
            ( XXL + 1.2516E-2 ) ) *
          ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) * 0.5 -
      ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( ( YZL - 7.41E-4 ) * ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) +
            ( ZZL + 4.815E-3 ) * ( cq35 - cq4 * sq35 ) -
            ( XZL - 1.196E-3 ) * ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) ) *
          0.5 +
      ( cly * mL - 3.127439544E-3 ) *
          ( ( cq35 - cq4 * sq35 ) *
                ( sq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                          cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                          cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
                  cq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                          cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                                  cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                          sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) -
            ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
                ( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
                  cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) +
            ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) *
                ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                  cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                          cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
                  cq4 * sq12 * 0.088 - cq1 * sq34 * 0.088 + cq23 * sq14 * 0.088 +
                  cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) +
            sq4 *
                ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
                  cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( sq3 * 0.088 - sq5 * 0.0825 + cq3 * sq5 * 0.0825 + sq35 * 0.384 ) ) -
      ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq35 * 4.354E-3 + cq3 * sq5 * 1.09E-4 + sq34 * 3.41E-4 + cq45 * sq3 * 1.09E-4 - cq4 * sq35 * 4.354E-3 ) *
          0.5 -
      ( cq13 + cq2 * sq1 * sq3 ) * ( cq3 * ( -8.626999999999999E-3 ) + cq4 * sq3 * 4.037E-3 + sq34 * 2.29E-4 ) * 0.5 +
      ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq35 * 1.09E-4 + cq3 * sq5 * 1.964E-3 - sq34 * 1.158E-3 + cq45 * sq3 * 1.964E-3 - cq4 * sq35 * 1.09E-4 ) *
          0.5 +
      ( cq3 * 7.796E-3 + sq3 * 2.5853E-2 ) * ( cq1 * sq3 - cq23 * sq1 ) * 0.5 +
      ( cq35 - cq4 * sq35 ) *
          ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.09E-4 -
            sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 4.354E-3 +
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 4.354E-3 + sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.09E-4 -
            cq4 * sq12 * 3.41E-4 + cq1 * sq34 * 3.41E-4 - cq23 * sq14 * 3.41E-4 ) *
          0.5 +
      ( cq35 - cq4 * sq35 ) *
          ( ( sq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) -
              cq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
                ( XZL - 1.196E-3 ) -
            ( ZZL + 4.815E-3 ) *
                ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) +
            ( YZL - 7.41E-4 ) * ( cq6 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) +
                                  sq6 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) +
                                          sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) ) *
          0.5 +
      ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) -
        sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq3 * sq5 + cq45 * sq3 ) * 2.3526756935E-2 +
      ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                cq2 * sq1 * sq3 * 0.0825 ) -
        sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq35 - cq4 * sq35 ) * 1.00241616695E-1 -
      cq4 *
          ( cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) -
            sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 2.892501 -
      sq4 *
          ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                    cq2 * sq1 * sq3 * 0.0825 ) +
            cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.225946 -
      sq34 *
          ( cq13 * 2.29E-4 - sq12 * sq4 * 2.117E-3 - cq14 * sq3 * 2.117E-3 + cq2 * sq1 * sq3 * 2.29E-4 +
            cq4 * sq12 * 2.9474E-2 - cq1 * sq34 * 2.9474E-2 + cq23 * cq4 * sq1 * 2.117E-3 + cq23 * sq14 * 2.9474E-2 ) *
          0.5 +
      cq2 * pcq3 * sq1 * 7.77507825633E-3 + cq1 * pcq3 * sq4 * 2.891981224275E-3 -
      cq4 * ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 2.3526756935E-2 +
      cq4 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.00241616695E-1 -
      cq4 * sq3 *
          ( cq13 * 4.037E-3 - sq12 * sq4 * 3.5549E-2 - cq14 * sq3 * 3.5549E-2 + cq2 * sq1 * sq3 * 4.037E-3 +
            cq4 * sq12 * 2.117E-3 - cq1 * sq34 * 2.117E-3 + cq23 * cq4 * sq1 * 3.5549E-2 + cq23 * sq14 * 2.117E-3 ) *
          0.5 +
      cq4 * ( cq13 + cq2 * sq1 * sq3 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.4653732538E-2 +
      sq4 * ( cq13 + cq2 * sq1 * sq3 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 5.034347249E-2 -
      sq34 *
          ( cq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 1.158E-3 +
            sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 3.41E-4 -
            cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 3.41E-4 + sq5 * ( cq13 + cq2 * sq1 * sq3 ) * 1.158E-3 +
            cq4 * sq12 * 5.433E-3 - cq1 * sq34 * 5.433E-3 + cq23 * sq14 * 5.433E-3 ) *
          0.5 -
      sq34 *
          ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
            cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          1.00241616695E-1 +
      sq34 *
          ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
            sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          2.3526756935E-2 -
      sq12 * ( cq3 * 8.640999999999999E-3 - sq3 * 1.332E-3 ) * 0.5 - cq23 * sq1 * 3.32621826471525E-2 -
      cq13 * sq3 * 7.77507825633E-3 - cq13 * sq4 * 2.891981224275E-3 +
      ( mL + 7.35522E-1 ) *
          ( sq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                    cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                    cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
            cq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                    cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                            cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                    sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
          ( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
            cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) +
      ( mL + 7.35522E-1 ) *
          ( cq6 * ( sq5 * ( sq12 * sq4 + cq14 * sq3 - cq23 * cq4 * sq1 ) * 0.088 -
                    cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 -
                    cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    sq12 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
            sq6 * ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                    cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) +
                            cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
                    sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
          ( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
            cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) -
      cq2 * sq1 * sq3 * 1.19235571141423E-1 + cq2 * sq14 * 6.730429031040001E-3 - cq4 * sq12 * 5.538582223460001E-3 -
      sq4 * ( mL + 7.35522E-1 ) * ( sq3 * 0.088 - sq5 * 0.0825 + cq3 * sq5 * 0.0825 + sq35 * 0.384 ) *
          ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
            cq4 * sq12 * 0.088 - cq1 * sq34 * 0.088 + cq23 * sq14 * 0.088 +
            cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) +
      cq5 * sq4 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) *
          2.3526756935E-2 -
      sq45 * ( cq4 * sq12 - cq1 * sq34 + cq23 * sq14 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.00241616695E-1 -
      cq2 * pcq3 * sq14 * 1.346085806208E-2 -
      cq5 * sq4 *
          ( cq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) -
            sq12 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.666555 -
      sq45 *
          ( sq5 * ( sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) +
                    cq4 * sq12 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) ) +
            cq5 * sq12 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.666555 +
      cq23 * sq1 * sq3 * 3.6189455156736E-2 - cq23 * sq14 * 5.538582223460001E-3 - cq34 * sq12 * 6.730429031040001E-3 +
      cq13 * sq34 * 1.346085806208E-2 - cq2 * sq1 * sq34 * 1.4459906121375E-3 + cq4 * sq12 * sq3 * 1.4459906121375E-3 +
      cq23 * sq1 * sq34 * 2.891981224275E-3 -
      cq4 * sq12 * sq3 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 5.034347249E-2 +
      sq12 * sq34 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 1.4653732538E-2;
  B[0][3] =
      cq13 * 8.640999999999999E-3 - cq1 * sq3 * 3.449E-3 - sq12 * 4.78240257230514E-1 +
      sq12 * sq3 * 2.543453644616475E-1 - sq12 * sq4 * 1.1198654185125E-2 - sq12 * sq5 * 7.698556162176E-2 -
      sq12 * ( ZZL + 4.815E-3 ) + cq1 * pcq4 * sq3 * 4.234E-3 - sq12 * ( mL + 7.35522E-1 ) * 1.6200625E-1 +
      pcq4 * sq12 * 4.995999999999997E-3 + pcq5 * sq12 * 2.39E-3 + cq13 * cq4 * 2.58333224672E-2 +
      cq23 * sq1 * 3.449E-3 - cq13 * sq4 * 1.481033294592E-3 - cq14 * sq3 * 5.5993270925625E-3 + cq15 * sq3 * 1.158E-3 +
      cq2 * sq1 * sq3 * 8.640999999999999E-3 - cq3 * sq12 * 4.19059345324423E-1 - cq4 * sq12 * 2.41786586877E-3 +
      cq1 * sq34 * 1.208932934385E-3 + cq5 * sq12 * 1.806854932608E-2 + cq1 * sq35 * 3.41E-4 -
      sq12 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) * 2.0 - cq3 * sq12 * ( mL * 1.21344E-1 + 8.9251181568E-2 ) -
      cq23 * pcq4 * sq1 * 4.234E-3 - cq13 * pcq5 * sq4 * 2.18E-4 - cq1 * pcq4 * cq5 * sq3 * 2.316E-3 -
      cq1 * pcq4 * sq35 * 6.82E-4 - pcq5 * sq12 * ( YYL + 1.0027E-2 ) +
      cq13 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) + pcq4 * sq12 * ( ZZL + 4.815E-3 ) +
      pcq5 * sq12 * ( ZZL + 4.815E-3 ) + cq5 * sq12 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) * 2.0 +
      cq15 * sq3 * ( XYL - 4.28E-4 ) + sq12 * sq3 * ( mL * 2.607E-2 + 1.917505854E-2 ) +
      cq3 * sq12 * ( mL * 6.806250000000001E-3 + 5.006146612500001E-3 ) + pcq5 * sq12 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      pcq4 * pcq5 * sq12 * 2.39E-3 + cq13 * cq45 * 3.41E-4 + sq12 * sq3 * ( mL * 3.168E-2 + 2.330133696E-2 ) +
      cq23 * cq4 * sq1 * 5.5993270925625E-3 - cq23 * cq5 * sq1 * 1.158E-3 - cq13 * cq4 * sq5 * 1.158E-3 +
      cq13 * cq5 * sq4 * 3.849278081088E-2 - cq6 * sq12 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq23 * sq14 * 1.208932934385E-3 + cq24 * sq1 * sq3 * 2.58333224672E-2 + cq34 * sq12 * 1.208932934385E-3 -
      cq23 * sq15 * 3.41E-4 + cq35 * sq12 * 7.434455191459999E-3 - cq14 * sq34 * 4.995999999999997E-3 +
      cq45 * sq12 * 1.6539866754675E-2 + cq13 * sq45 * 9.03427466304E-3 - cq15 * sq34 * 8.269933377337502E-3 +
      sq12 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq2 * sq1 * sq34 * 1.481033294592E-3 +
      cq3 * sq12 * sq4 * 5.5993270925625E-3 + cq4 * sq12 * sq3 * 4.630579482008E-3 -
      cq3 * sq12 * sq5 * 3.167635087562E-2 + cq4 * sq12 * sq4 * 4.234E-3 - cq5 * sq12 * sq3 * 1.9409574471375E-3 +
      cq4 * sq12 * sq5 * 3.881914894275E-3 - cq1 * sq34 * sq5 * 1.9409574471375E-3 + cq5 * sq12 * sq5 * 2.18E-4 -
      pcq4 * sq12 * ( XXL + 1.2516E-2 ) + sq12 * sq34 * 2.14471195303E-2 + sq12 * sq35 * 8.269933377337502E-3 +
      pcq5 * cq6 * sq12 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 + cq23 * pcq4 * sq15 * 6.82E-4 +
      cq14 * pcq5 * sq34 * 2.39E-3 - cq6 * sq12 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) * 2.0 +
      cq45 * sq12 * ( mL * 7.26E-3 + 5.33988972E-3 ) * 2.0 - cq15 * sq34 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      pcq5 * sq12 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq2 * pcq5 * sq1 * sq34 * 2.18E-4 -
      pcq4 * cq5 * sq12 * sq5 * 2.18E-4 + pcq4 * pcq6 * sq12 * ( XXL + 1.2516E-2 ) -
      pcq5 * pcq6 * sq12 * ( XXL + 1.2516E-2 ) + cq14 * cq6 * sq3 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq14 * sq36 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + sq12 * sq35 * ( mL * 7.26E-3 + 5.33988972E-3 ) +
      cq2 * sq1 * sq34 * ( clz * mL * 0.088 + 3.986923479792E-3 ) +
      cq5 * sq12 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq13 * sq45 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) - cq23 * cq5 * sq1 * ( XYL - 4.28E-4 ) -
      cq13 * cq4 * sq5 * ( XYL - 4.28E-4 ) + cq6 * sq12 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) * 2.0 -
      cq13 * cq4 * cq6 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
      sq12 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) * 2.0 - cq13 * cq6 * sq4 * ( XZL - 1.196E-3 ) +
      cq14 * sq34 * ( XXL + 1.2516E-2 ) - cq1 * sq35 * sq6 * ( XZL - 1.196E-3 ) +
      pcq4 * pcq5 * sq12 * ( YYL + 1.0027E-2 ) - pcq4 * pcq6 * sq12 * ( YYL + 1.0027E-2 ) +
      pcq5 * pcq6 * sq12 * ( YYL + 1.0027E-2 ) - cq3 * sq12 * sq5 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) -
      cq13 * pcq5 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      sq12 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) * 2.0 +
      cq15 * cq6 * sq3 * ( cly * mL * 0.088 - 2.75214679872E-4 ) - pcq4 * pcq5 * sq12 * ( ZZL + 4.815E-3 ) -
      cq1 * pcq4 * cq5 * sq3 * ( XYL - 4.28E-4 ) * 2.0 - cq15 * pcq6 * sq3 * ( XYL - 4.28E-4 ) * 2.0 +
      cq15 * sq36 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + cq23 * cq4 * sq14 * 4.995999999999997E-3 +
      cq23 * cq5 * sq14 * 8.269933377337502E-3 + cq24 * cq5 * sq1 * sq3 * 3.41E-4 -
      cq34 * cq5 * sq12 * 8.269933377337502E-3 + cq13 * cq5 * sq45 * 2.39E-3 + cq23 * sq14 * sq5 * 1.9409574471375E-3 -
      cq24 * sq1 * sq35 * 1.158E-3 + cq25 * sq1 * sq34 * 3.849278081088E-2 - cq34 * sq12 * sq5 * 1.9409574471375E-3 -
      cq45 * sq12 * sq3 * 3.167635087562E-2 - cq45 * sq12 * sq4 * 2.316E-3 -
      pcq4 * pcq5 * sq12 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      cq5 * sq12 * sq3 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq4 * sq12 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) * 2.0 -
      cq1 * sq34 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq13 * cq5 * sq4 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) + cq2 * sq1 * sq34 * sq5 * 9.03427466304E-3 -
      cq4 * sq12 * sq35 * 7.434455191459999E-3 - cq4 * sq12 * sq45 * 6.82E-4 - cq14 * sq34 * ( ZZL + 4.815E-3 ) +
      cq13 * sq46 * ( YZL - 7.41E-4 ) - cq16 * sq35 * ( YZL - 7.41E-4 ) -
      cq13 * cq4 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq35 * sq12 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) + cq23 * pcq4 * cq5 * sq1 * 2.316E-3 +
      cq14 * pcq5 * sq34 * ( ZZL + 4.815E-3 ) - cq36 * sq12 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq45 * sq12 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) * 2.0 +
      cq15 * sq34 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq3 * sq12 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq6 * sq12 * sq35 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) - cq13 * cq45 * sq6 * ( XZL - 1.196E-3 ) -
      cq13 * pcq5 * sq46 * ( YZL - 7.41E-4 ) * 2.0 + cq1 * pcq4 * cq6 * sq35 * ( YZL - 7.41E-4 ) * 2.0 +
      cq2 * sq1 * sq34 * sq5 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) - cq24 * sq1 * sq35 * ( XYL - 4.28E-4 ) -
      cq45 * sq12 * sq4 * ( XYL - 4.28E-4 ) * 2.0 + cq14 * pcq5 * sq34 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      sq12 * sq35 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq23 * cq4 * sq14 * ( XXL + 1.2516E-2 ) -
      cq15 * cq6 * sq36 * ( XXL + 1.2516E-2 ) - cq24 * cq6 * sq1 * sq3 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq13 * cq5 * sq46 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      pcq4 * pcq5 * cq6 * sq12 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq26 * sq1 * sq34 * ( XZL - 1.196E-3 ) + cq23 * sq15 * sq6 * ( XZL - 1.196E-3 ) -
      cq5 * cq6 * sq12 * sq5 * ( XZL - 1.196E-3 ) * 2.0 - cq45 * sq12 * sq3 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) +
      pcq4 * pcq5 * sq12 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      pcq4 * pcq5 * pcq6 * sq12 * ( XXL + 1.2516E-2 ) -
      cq23 * cq5 * cq6 * sq1 * ( cly * mL * 0.088 - 2.75214679872E-4 ) -
      cq13 * cq4 * cq6 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) -
      cq2 * pcq5 * sq1 * sq34 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      pcq4 * cq5 * sq12 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq23 * pcq4 * cq5 * sq1 * ( XYL - 4.28E-4 ) * 2.0 + cq23 * cq5 * pcq6 * sq1 * ( XYL - 4.28E-4 ) * 2.0 +
      cq13 * cq4 * pcq6 * sq5 * ( XYL - 4.28E-4 ) * 2.0 - cq36 * sq12 * sq5 * ( clx * mL * 0.316 + 2.444413220184E-3 ) -
      cq23 * cq5 * sq16 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      cq13 * cq4 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + cq13 * pcq5 * cq6 * sq4 * ( XZL - 1.196E-3 ) * 2.0 -
      cq6 * sq12 * sq34 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) +
      cq3 * sq12 * sq56 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) - cq13 * cq5 * sq45 * ( YYL + 1.0027E-2 ) +
      cq15 * cq6 * sq36 * ( YYL + 1.0027E-2 ) + cq14 * cq5 * sq34 * sq5 * 2.18E-4 -
      pcq4 * cq6 * sq12 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + pcq5 * cq6 * sq12 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq14 * pcq6 * sq34 * ( XXL + 1.2516E-2 ) + cq23 * sq14 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      cq34 * sq12 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      sq12 * sq34 * sq6 * ( clx * mL * 0.316 + 2.444413220184E-3 ) + cq25 * sq1 * sq34 * sq5 * 2.39E-3 +
      cq1 * pcq4 * sq35 * sq6 * ( XZL - 1.196E-3 ) * 2.0 - cq13 * cq45 * cq6 * ( YZL - 7.41E-4 ) -
      pcq4 * pcq5 * pcq6 * sq12 * ( YYL + 1.0027E-2 ) + cq23 * cq4 * sq14 * ( ZZL + 4.815E-3 ) +
      cq13 * cq5 * sq45 * ( ZZL + 4.815E-3 ) + cq25 * sq1 * sq34 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
      cq23 * cq6 * sq15 * ( YZL - 7.41E-4 ) + cq13 * cq5 * cq6 * sq4 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      cq1 * pcq4 * cq5 * cq6 * sq3 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq1 * pcq4 * cq5 * pcq6 * sq3 * ( XYL - 4.28E-4 ) * 4.0 + cq13 * cq5 * sq45 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      cq1 * pcq4 * cq5 * sq36 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 - cq23 * cq4 * pcq5 * sq14 * 2.39E-3 +
      cq2 * sq1 * sq34 * sq6 * ( YZL - 7.41E-4 ) + cq5 * sq12 * sq56 * ( YZL - 7.41E-4 ) * 2.0 -
      cq24 * sq1 * sq36 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq23 * cq5 * sq14 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq34 * cq5 * sq12 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      cq14 * pcq5 * sq34 * ( YYL + 1.0027E-2 ) + cq14 * pcq6 * sq34 * ( YYL + 1.0027E-2 ) -
      cq4 * sq12 * sq35 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) -
      cq23 * cq4 * cq6 * sq1 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq23 * cq4 * sq16 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq45 * cq6 * sq12 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) * 2.0 -
      cq15 * cq6 * sq34 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq14 * cq6 * sq34 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq23 * cq4 * pcq5 * sq14 * ( mL + 7.35522E-1 ) * 7.744E-3 + cq23 * cq5 * cq6 * sq16 * ( XXL + 1.2516E-2 ) +
      cq13 * cq4 * cq6 * sq56 * ( XXL + 1.2516E-2 ) - cq24 * cq5 * sq1 * sq36 * ( XZL - 1.196E-3 ) -
      cq2 * pcq5 * sq1 * sq34 * sq6 * ( YZL - 7.41E-4 ) * 2.0 - pcq4 * cq5 * sq12 * sq56 * ( YZL - 7.41E-4 ) * 2.0 +
      cq14 * pcq5 * pcq6 * sq34 * ( YYL + 1.0027E-2 ) -
      cq25 * sq1 * sq34 * sq6 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
      cq4 * sq12 * sq45 * sq6 * ( XZL - 1.196E-3 ) * 2.0 -
      cq45 * cq6 * sq12 * sq3 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq13 * cq5 * cq6 * sq45 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq45 * sq12 * sq36 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) - cq23 * cq5 * cq6 * sq16 * ( YYL + 1.0027E-2 ) -
      cq13 * cq4 * cq6 * sq56 * ( YYL + 1.0027E-2 ) -
      cq24 * cq6 * sq1 * sq35 * ( cly * mL * 0.088 - 2.75214679872E-4 ) -
      cq45 * cq6 * sq12 * sq4 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq13 * cq5 * sq45 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq23 * cq45 * sq14 * sq5 * 2.18E-4 +
      cq24 * pcq6 * sq1 * sq35 * ( XYL - 4.28E-4 ) * 2.0 + cq45 * pcq6 * sq12 * sq4 * ( XYL - 4.28E-4 ) * 4.0 +
      cq23 * cq4 * pcq6 * sq14 * ( XXL + 1.2516E-2 ) - cq13 * cq5 * pcq6 * sq45 * ( XXL + 1.2516E-2 ) +
      cq1 * pcq4 * cq5 * cq6 * sq36 * ( XXL + 1.2516E-2 ) * 2.0 -
      cq24 * sq1 * sq35 * sq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      cq45 * sq12 * sq46 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq23 * pcq4 * sq15 * sq6 * ( XZL - 1.196E-3 ) * 2.0 + cq2 * pcq5 * cq6 * sq1 * sq34 * ( XZL - 1.196E-3 ) * 2.0 +
      pcq4 * cq5 * cq6 * sq12 * sq5 * ( XZL - 1.196E-3 ) * 2.0 - cq25 * sq1 * sq34 * sq5 * ( YYL + 1.0027E-2 ) -
      cq24 * cq5 * cq6 * sq1 * sq3 * ( YZL - 7.41E-4 ) +
      cq23 * pcq4 * cq5 * cq6 * sq1 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq25 * sq1 * sq34 * sq5 * ( ZZL + 4.815E-3 ) - cq23 * pcq4 * cq5 * pcq6 * sq1 * ( XYL - 4.28E-4 ) * 4.0 +
      cq23 * pcq4 * cq5 * sq16 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq14 * pcq5 * cq6 * sq34 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq4 * cq6 * sq12 * sq45 * ( YZL - 7.41E-4 ) * 2.0 +
      cq25 * cq6 * sq1 * sq34 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq23 * cq4 * pcq5 * sq14 * ( YYL + 1.0027E-2 ) - cq23 * cq4 * pcq6 * sq14 * ( YYL + 1.0027E-2 ) +
      cq13 * cq5 * pcq6 * sq45 * ( YYL + 1.0027E-2 ) - cq1 * pcq4 * cq5 * cq6 * sq36 * ( YYL + 1.0027E-2 ) * 2.0 -
      cq14 * pcq5 * sq34 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      pcq4 * pcq5 * cq6 * sq12 * sq6 * ( XYL - 4.28E-4 ) * 2.0 - cq14 * pcq5 * pcq6 * sq34 * ( XXL + 1.2516E-2 ) +
      cq25 * sq1 * sq34 * sq5 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq23 * cq5 * cq6 * sq14 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      cq34 * cq5 * cq6 * sq12 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq14 * cq5 * sq34 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      cq23 * cq4 * pcq5 * sq14 * ( ZZL + 4.815E-3 ) -
      cq23 * cq5 * sq14 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq34 * cq5 * sq12 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq23 * pcq4 * cq6 * sq15 * ( YZL - 7.41E-4 ) * 2.0 + cq14 * pcq5 * cq6 * sq34 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq23 * pcq4 * cq5 * cq6 * sq16 * ( XXL + 1.2516E-2 ) * 2.0 +
      cq25 * cq6 * sq1 * sq34 * sq5 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq24 * cq6 * sq1 * sq35 * sq6 * ( YYL + 1.0027E-2 ) - cq45 * cq6 * sq12 * sq46 * ( YYL + 1.0027E-2 ) * 2.0 -
      cq25 * sq1 * sq34 * sq56 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq25 * pcq6 * sq1 * sq34 * sq5 * ( XXL + 1.2516E-2 ) -
      cq23 * cq4 * pcq5 * cq6 * sq14 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq14 * cq5 * sq34 * sq56 * ( YZL - 7.41E-4 ) * 2.0 + cq23 * pcq4 * cq5 * cq6 * sq16 * ( YYL + 1.0027E-2 ) * 2.0 +
      cq23 * cq4 * pcq5 * sq14 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq23 * cq4 * pcq5 * pcq6 * sq14 * ( XXL + 1.2516E-2 ) + cq25 * pcq6 * sq1 * sq34 * sq5 * ( YYL + 1.0027E-2 ) -
      cq23 * cq45 * sq14 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      cq23 * cq4 * cq6 * sq14 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
      cq13 * cq5 * cq6 * sq45 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq14 * cq5 * cq6 * sq34 * sq5 * ( XZL - 1.196E-3 ) * 2.0 - cq23 * cq4 * pcq5 * pcq6 * sq14 * ( YYL + 1.0027E-2 ) +
      cq24 * cq6 * sq1 * sq35 * sq6 * ( XXL + 1.2516E-2 ) + cq45 * cq6 * sq12 * sq46 * ( XXL + 1.2516E-2 ) * 2.0 -
      cq23 * cq4 * pcq5 * cq6 * sq14 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq23 * cq45 * sq14 * sq56 * ( YZL - 7.41E-4 ) * 2.0 + cq23 * cq45 * cq6 * sq14 * sq5 * ( XZL - 1.196E-3 ) * 2.0 +
      cq25 * cq6 * sq1 * sq34 * sq56 * ( XYL - 4.28E-4 ) * 2.0;
  B[0][4] =
      cq13 * 1.0591E-2 - cq14 * 1.208932934385E-3 - cq1 * sq4 * 5.5993270925625E-3 + sq12 * sq4 * 1.481033294592E-3 +
      cq13 * pcq5 * 2.39E-3 + cq13 * cq4 * 1.208932934385E-3 + cq14 * cq5 * 8.269933377337502E-3 +
      cq13 * ( YYL + 1.0027E-2 ) - cq24 * sq1 * 4.630579482008E-3 + cq13 * sq4 * 5.5993270925625E-3 +
      cq14 * sq3 * 1.481033294592E-3 + cq14 * sq5 * 1.9409574471375E-3 + cq2 * sq1 * sq3 * 1.0591E-2 -
      cq2 * sq14 * 2.14471195303E-2 - cq4 * sq12 * 2.58333224672E-2 + cq1 * sq34 * 2.58333224672E-2 +
      cq14 * cq5 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq13 * pcq5 * ( YYL + 1.0027E-2 ) -
      cq13 * pcq6 * ( YYL + 1.0027E-2 ) + cq14 * pcq5 * sq3 * 2.18E-4 + cq2 * pcq5 * sq1 * sq3 * 2.39E-3 -
      cq14 * sq3 * ( clz * mL * 0.088 + 3.986923479792E-3 ) + pcq5 * sq12 * sq4 * 2.18E-4 +
      cq13 * pcq5 * ( ZZL + 4.815E-3 ) + cq16 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq1 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) - sq12 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) +
      cq13 * pcq5 * ( mL + 7.35522E-1 ) * 7.744E-3 - cq13 * cq45 * 8.269933377337502E-3 -
      cq23 * cq4 * sq1 * 1.481033294592E-3 + cq24 * cq5 * sq1 * 3.167635087562E-2 -
      cq13 * cq4 * sq5 * 1.9409574471375E-3 - cq14 * cq5 * sq3 * 3.849278081088E-2 + cq13 * cq5 * sq5 * 2.18E-4 +
      cq13 * pcq6 * ( XXL + 1.2516E-2 ) + cq14 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      cq23 * sq14 * 2.58333224672E-2 + cq24 * sq1 * sq3 * 1.208932934385E-3 - cq34 * sq12 * 2.14471195303E-2 +
      cq24 * sq15 * 7.434455191459999E-3 - cq45 * sq12 * 3.41E-4 - cq14 * sq35 * 9.03427466304E-3 +
      cq15 * sq34 * 3.41E-4 + cq2 * sq1 * sq3 * ( YYL + 1.0027E-2 ) + cq2 * sq1 * sq34 * 5.5993270925625E-3 +
      cq3 * sq12 * sq4 * 4.630579482008E-3 + cq4 * sq12 * sq3 * 5.5993270925625E-3 + cq4 * sq12 * sq5 * 1.158E-3 -
      cq5 * sq12 * sq4 * 3.849278081088E-2 - cq1 * sq34 * sq5 * 1.158E-3 - sq12 * sq34 * 1.208932934385E-3 -
      sq12 * sq45 * 9.03427466304E-3 + cq14 * cq5 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      sq12 * sq46 * ( YZL - 7.41E-4 ) + cq4 * sq12 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      cq1 * sq34 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) - cq2 * pcq5 * sq1 * sq3 * ( YYL + 1.0027E-2 ) -
      cq2 * pcq6 * sq1 * sq3 * ( YYL + 1.0027E-2 ) + cq23 * cq4 * sq1 * ( clz * mL * 0.088 + 3.986923479792E-3 ) +
      cq13 * cq5 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      cq13 * cq6 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq14 * cq5 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq13 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq2 * pcq5 * sq1 * sq3 * ( ZZL + 4.815E-3 ) -
      cq14 * sq35 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) - cq13 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
      cq14 * cq6 * sq3 * ( XZL - 1.196E-3 ) + cq13 * pcq5 * pcq6 * ( YYL + 1.0027E-2 ) +
      cq24 * cq5 * sq1 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) -
      sq12 * sq45 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) + cq4 * sq12 * sq5 * ( XYL - 4.28E-4 ) -
      cq1 * sq34 * sq5 * ( XYL - 4.28E-4 ) + cq2 * pcq5 * sq1 * sq3 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq4 * cq6 * sq12 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq16 * sq34 * ( cly * mL * 0.384 - 1.200936784896E-3 ) + cq6 * sq12 * sq4 * ( XZL - 1.196E-3 ) +
      cq14 * pcq5 * sq3 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq26 * sq14 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) +
      pcq5 * sq12 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 + cq23 * cq45 * sq1 * 3.849278081088E-2 -
      cq13 * cq4 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq2 * sq14 * sq6 * ( clx * mL * 0.316 + 2.444413220184E-3 ) + cq23 * cq4 * sq15 * 9.03427466304E-3 -
      cq23 * cq5 * sq14 * 3.41E-4 - cq24 * cq5 * sq1 * sq3 * 8.269933377337502E-3 - cq14 * cq5 * sq35 * 2.39E-3 +
      cq23 * sq14 * sq5 * 1.158E-3 - cq24 * sq1 * sq35 * 1.9409574471375E-3 - cq35 * sq12 * sq4 * 3.167635087562E-2 +
      cq25 * sq1 * sq35 * 2.18E-4 + cq2 * pcq6 * sq1 * sq3 * ( XXL + 1.2516E-2 ) -
      cq14 * cq5 * sq3 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) - cq3 * sq12 * sq45 * 7.434455191459999E-3 +
      cq5 * sq12 * sq34 * 8.269933377337502E-3 - cq5 * sq12 * sq45 * 2.39E-3 + sq12 * sq34 * sq5 * 1.9409574471375E-3 -
      cq5 * sq12 * sq4 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
      cq13 * pcq5 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 - cq14 * sq36 * ( YZL - 7.41E-4 ) -
      cq13 * cq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      cq13 * pcq5 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq24 * sq15 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) - cq23 * cq4 * pcq5 * sq1 * 2.18E-4 -
      cq13 * pcq5 * pcq6 * ( XXL + 1.2516E-2 ) + cq5 * sq12 * sq34 * ( mL * 7.26E-3 + 5.33988972E-3 ) +
      cq25 * sq1 * sq35 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq23 * cq4 * sq15 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) -
      cq26 * sq1 * sq34 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq4 * cq6 * sq12 * sq3 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq2 * sq1 * sq34 * sq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      cq4 * sq12 * sq36 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) - cq23 * cq4 * cq6 * sq1 * ( XZL - 1.196E-3 ) -
      cq13 * cq5 * cq6 * sq5 * ( XZL - 1.196E-3 ) * 2.0 + cq14 * pcq5 * sq36 * ( YZL - 7.41E-4 ) * 2.0 +
      cq23 * sq14 * sq5 * ( XYL - 4.28E-4 ) - cq26 * sq1 * sq36 * ( XYL - 4.28E-4 ) * 2.0 +
      cq23 * cq6 * sq14 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
      cq14 * cq5 * sq36 * ( cly * mL * 0.384 - 1.200936784896E-3 ) + cq45 * sq12 * sq6 * ( XZL - 1.196E-3 ) -
      cq15 * sq34 * sq6 * ( XZL - 1.196E-3 ) + pcq5 * sq12 * sq46 * ( YZL - 7.41E-4 ) * 2.0 +
      cq2 * pcq5 * pcq6 * sq1 * sq3 * ( YYL + 1.0027E-2 ) - cq35 * sq12 * sq4 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) -
      cq23 * cq4 * pcq5 * sq1 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq5 * sq12 * sq46 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
      cq24 * cq5 * cq6 * sq1 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq34 * cq6 * sq12 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) -
      cq24 * cq5 * sq16 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) +
      cq13 * pcq5 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + cq34 * sq12 * sq6 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq23 * cq45 * sq15 * 2.39E-3 - cq14 * pcq5 * cq6 * sq3 * ( XZL - 1.196E-3 ) * 2.0 +
      cq14 * cq5 * sq35 * ( YYL + 1.0027E-2 ) + cq4 * cq6 * sq12 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) -
      cq16 * sq34 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) - cq4 * pcq6 * sq12 * sq5 * ( XYL - 4.28E-4 ) * 2.0 +
      cq1 * pcq6 * sq34 * sq5 * ( XYL - 4.28E-4 ) * 2.0 -
      cq24 * sq1 * sq35 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq23 * cq45 * sq1 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
      cq4 * sq12 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      cq1 * sq34 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) - pcq5 * cq6 * sq12 * sq4 * ( XZL - 1.196E-3 ) * 2.0 +
      cq5 * sq12 * sq45 * ( YYL + 1.0027E-2 ) - cq14 * cq5 * sq35 * ( ZZL + 4.815E-3 ) +
      sq12 * sq34 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) + cq23 * cq4 * sq16 * ( YZL - 7.41E-4 ) +
      cq45 * cq6 * sq12 * ( YZL - 7.41E-4 ) - cq15 * cq6 * sq34 * ( YZL - 7.41E-4 ) +
      cq13 * cq5 * sq56 * ( YZL - 7.41E-4 ) * 2.0 - cq14 * cq5 * cq6 * sq3 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      cq5 * sq12 * sq45 * ( ZZL + 4.815E-3 ) - cq14 * cq5 * sq35 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq2 * pcq5 * cq6 * sq1 * sq3 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq13 * cq45 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq23 * sq14 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      cq5 * cq6 * sq12 * sq4 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      cq24 * cq5 * sq1 * sq3 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      cq2 * pcq5 * sq1 * sq36 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq3 * sq12 * sq45 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) -
      cq2 * pcq5 * pcq6 * sq1 * sq3 * ( XXL + 1.2516E-2 ) +
      cq13 * cq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq5 * sq12 * sq45 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      cq5 * sq12 * sq34 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq23 * cq45 * sq16 * ( cly * mL * 0.384 - 1.200936784896E-3 ) + cq23 * cq5 * sq14 * sq6 * ( XZL - 1.196E-3 ) -
      cq25 * cq6 * sq1 * sq35 * ( XZL - 1.196E-3 ) * 2.0 - cq4 * cq6 * sq12 * sq56 * ( XXL + 1.2516E-2 ) +
      cq16 * sq34 * sq56 * ( XXL + 1.2516E-2 ) - cq35 * cq6 * sq12 * sq4 * ( clx * mL * 0.316 + 2.444413220184E-3 ) -
      cq14 * cq5 * cq6 * sq35 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq23 * cq4 * pcq5 * cq6 * sq1 * ( XZL - 1.196E-3 ) * 2.0 +
      cq35 * sq12 * sq46 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) - cq23 * cq45 * sq15 * ( YYL + 1.0027E-2 ) +
      cq23 * cq6 * sq14 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) +
      cq14 * cq5 * sq35 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq23 * pcq6 * sq14 * sq5 * ( XYL - 4.28E-4 ) * 2.0 + cq2 * pcq5 * cq6 * sq1 * sq36 * ( XYL - 4.28E-4 ) * 2.0 +
      cq14 * cq5 * pcq6 * sq35 * ( XXL + 1.2516E-2 ) + cq23 * sq14 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      cq5 * cq6 * sq12 * sq45 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq4 * cq6 * sq12 * sq56 * ( YYL + 1.0027E-2 ) - cq16 * sq34 * sq56 * ( YYL + 1.0027E-2 ) +
      cq5 * sq12 * sq45 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq23 * cq45 * sq15 * ( ZZL + 4.815E-3 ) + cq5 * pcq6 * sq12 * sq45 * ( XXL + 1.2516E-2 ) +
      cq23 * cq5 * cq6 * sq14 * ( YZL - 7.41E-4 ) + cq23 * cq45 * cq6 * sq1 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq23 * cq45 * sq15 * ( mL + 7.35522E-1 ) * 7.744E-3 + cq25 * sq1 * sq35 * sq6 * ( YZL - 7.41E-4 ) * 2.0 -
      cq14 * cq5 * pcq6 * sq35 * ( YYL + 1.0027E-2 ) -
      cq24 * cq5 * cq6 * sq1 * sq3 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      cq5 * pcq6 * sq12 * sq45 * ( YYL + 1.0027E-2 ) +
      cq24 * cq5 * sq1 * sq36 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq5 * cq6 * sq12 * sq34 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      cq23 * cq4 * pcq5 * sq16 * ( YZL - 7.41E-4 ) * 2.0 +
      cq23 * cq45 * cq6 * sq15 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq23 * cq45 * sq15 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq23 * cq45 * pcq6 * sq15 * ( XXL + 1.2516E-2 ) + cq23 * cq6 * sq14 * sq56 * ( YYL + 1.0027E-2 ) +
      cq23 * cq45 * pcq6 * sq15 * ( YYL + 1.0027E-2 ) - cq14 * cq5 * cq6 * sq35 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq5 * cq6 * sq12 * sq45 * sq6 * ( XYL - 4.28E-4 ) * 2.0 - cq23 * cq6 * sq14 * sq56 * ( XXL + 1.2516E-2 ) +
      cq23 * cq45 * cq6 * sq15 * sq6 * ( XYL - 4.28E-4 ) * 2.0;
  B[0][5] =
      sq12 * sq5 * 1.9409574471375E-3 + cq13 * cq5 * 3.41E-4 - cq13 * sq5 * 1.158E-3 + cq15 * sq4 * 1.9409574471375E-3 -
      cq4 * sq12 * 5.433E-3 + cq1 * sq34 * 5.433E-3 + cq5 * sq12 * 8.269933377337502E-3 -
      cq1 * sq45 * 8.269933377337502E-3 - cq4 * sq12 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq1 * sq34 * ( mL + 7.35522E-1 ) * 7.744E-3 + cq5 * sq12 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      cq1 * sq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq13 * sq5 * ( XYL - 4.28E-4 ) -
      cq4 * sq12 * ( XXL + 1.2516E-2 ) + cq1 * sq34 * ( XXL + 1.2516E-2 ) - cq13 * cq5 * sq4 * 1.9409574471375E-3 -
      cq14 * cq5 * sq3 * 1.158E-3 + cq15 * sq4 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) - cq23 * sq14 * 5.433E-3 +
      cq25 * sq1 * sq3 * 3.41E-4 - cq35 * sq12 * 8.269933377337502E-3 + cq25 * sq14 * 7.434455191459999E-3 +
      cq45 * sq12 * 9.03427466304E-3 + cq13 * sq45 * 8.269933377337502E-3 - cq14 * sq35 * 3.41E-4 -
      cq15 * sq34 * 9.03427466304E-3 - cq2 * sq1 * sq35 * 1.158E-3 - cq3 * sq12 * sq5 * 1.9409574471375E-3 -
      cq5 * sq12 * sq3 * 3.167635087562E-2 - cq2 * sq14 * sq5 * 3.167635087562E-2 -
      cq4 * sq12 * sq5 * 3.849278081088E-2 - cq5 * sq12 * sq4 * 1.158E-3 + cq1 * sq34 * sq5 * 3.849278081088E-2 +
      sq12 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) - sq12 * sq35 * 7.434455191459999E-3 -
      sq12 * sq45 * 3.41E-4 - cq35 * sq12 * ( mL * 7.26E-3 + 5.33988972E-3 ) +
      cq13 * sq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq4 * pcq6 * sq12 * ( YYL + 1.0027E-2 ) +
      cq1 * pcq6 * sq34 * ( YYL + 1.0027E-2 ) - sq12 * sq35 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) +
      cq5 * cq6 * sq12 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      cq16 * sq45 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq45 * sq12 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) -
      cq15 * sq34 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) - cq14 * cq5 * sq3 * ( XYL - 4.28E-4 ) -
      cq5 * sq12 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq1 * sq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq13 * cq5 * sq6 * ( XZL - 1.196E-3 ) -
      cq2 * sq1 * sq35 * ( XYL - 4.28E-4 ) - cq5 * sq12 * sq4 * ( XYL - 4.28E-4 ) - cq23 * sq14 * ( XXL + 1.2516E-2 ) -
      cq5 * sq12 * sq3 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) -
      cq2 * sq14 * sq5 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) -
      cq13 * cq6 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + cq23 * cq45 * sq1 * 1.158E-3 +
      cq13 * pcq6 * sq5 * ( XYL - 4.28E-4 ) * 2.0 - cq13 * cq5 * sq4 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      cq4 * cq6 * sq12 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq16 * sq34 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq13 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + cq23 * cq4 * sq15 * 3.41E-4 +
      cq23 * cq5 * sq14 * 9.03427466304E-3 + cq34 * cq5 * sq12 * 7.434455191459999E-3 +
      cq4 * sq12 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq1 * sq34 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq23 * sq14 * sq5 * 3.849278081088E-2 -
      cq25 * sq1 * sq34 * 1.9409574471375E-3 - cq34 * sq12 * sq5 * 3.167635087562E-2 -
      cq45 * sq12 * sq3 * 1.9409574471375E-3 + cq4 * pcq6 * sq12 * ( XXL + 1.2516E-2 ) -
      cq1 * pcq6 * sq34 * ( XXL + 1.2516E-2 ) - cq3 * sq12 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq2 * sq1 * sq34 * sq5 * 8.269933377337502E-3 + cq4 * sq12 * sq35 * 8.269933377337502E-3 -
      cq13 * cq5 * cq6 * ( YZL - 7.41E-4 ) - cq4 * sq12 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
      cq1 * sq34 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
      cq25 * sq14 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) - cq23 * sq14 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq2 * sq1 * sq34 * sq5 * ( mL * 7.26E-3 + 5.33988972E-3 ) + cq4 * sq12 * sq35 * ( mL * 7.26E-3 + 5.33988972E-3 ) +
      cq23 * cq5 * sq14 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) + cq23 * cq45 * sq1 * ( XYL - 4.28E-4 ) +
      cq35 * sq12 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq13 * sq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq4 * cq6 * sq12 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
      cq16 * sq34 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + cq13 * cq6 * sq56 * ( XXL + 1.2516E-2 ) -
      cq25 * sq1 * sq36 * ( XZL - 1.196E-3 ) + cq14 * sq35 * sq6 * ( XZL - 1.196E-3 ) -
      cq34 * sq12 * sq5 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) +
      cq4 * sq12 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq1 * sq34 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) + sq12 * sq45 * sq6 * ( XZL - 1.196E-3 ) -
      cq14 * cq5 * cq6 * sq3 * ( cly * mL * 0.088 - 2.75214679872E-4 ) +
      cq14 * cq5 * pcq6 * sq3 * ( XYL - 4.28E-4 ) * 2.0 -
      cq5 * cq6 * sq12 * sq3 * ( clx * mL * 0.316 + 2.444413220184E-3 ) -
      cq26 * sq14 * sq5 * ( clx * mL * 0.316 + 2.444413220184E-3 ) -
      cq23 * cq6 * sq14 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq14 * cq5 * sq36 * ( clx * mL * 0.088 + 6.80722668912E-4 ) +
      cq5 * sq12 * sq36 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) +
      cq2 * sq14 * sq56 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) - cq13 * cq6 * sq56 * ( YYL + 1.0027E-2 ) +
      cq23 * sq14 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq26 * sq1 * sq35 * ( cly * mL * 0.088 - 2.75214679872E-4 ) -
      cq5 * cq6 * sq12 * sq4 * ( cly * mL * 0.088 - 2.75214679872E-4 ) +
      cq2 * pcq6 * sq1 * sq35 * ( XYL - 4.28E-4 ) * 2.0 + cq5 * pcq6 * sq12 * sq4 * ( XYL - 4.28E-4 ) * 2.0 +
      cq23 * pcq6 * sq14 * ( XXL + 1.2516E-2 ) - cq25 * sq1 * sq34 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      cq45 * sq12 * sq3 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      cq2 * sq1 * sq35 * sq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      cq5 * sq12 * sq46 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      cq23 * sq14 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) - cq25 * cq6 * sq1 * sq3 * ( YZL - 7.41E-4 ) +
      cq14 * cq6 * sq35 * ( YZL - 7.41E-4 ) + cq34 * cq5 * sq12 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) +
      cq6 * sq12 * sq45 * ( YZL - 7.41E-4 ) - cq4 * cq6 * sq12 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq16 * sq34 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) - cq23 * pcq6 * sq14 * ( YYL + 1.0027E-2 ) -
      cq35 * cq6 * sq12 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq13 * cq6 * sq45 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) - cq23 * cq6 * sq14 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq2 * sq1 * sq34 * sq56 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq4 * sq12 * sq35 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq14 * cq5 * cq6 * sq36 * ( XXL + 1.2516E-2 ) - cq23 * cq4 * sq15 * sq6 * ( XZL - 1.196E-3 ) +
      cq26 * sq1 * sq35 * sq6 * ( XXL + 1.2516E-2 ) + cq5 * cq6 * sq12 * sq46 * ( XXL + 1.2516E-2 ) +
      cq23 * sq14 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
      cq23 * cq45 * cq6 * sq1 * ( cly * mL * 0.088 - 2.75214679872E-4 ) -
      cq23 * cq45 * pcq6 * sq1 * ( XYL - 4.28E-4 ) * 2.0 -
      cq34 * cq6 * sq12 * sq5 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq23 * cq45 * sq16 * ( clx * mL * 0.088 + 6.80722668912E-4 ) +
      cq34 * sq12 * sq56 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) - cq14 * cq5 * cq6 * sq36 * ( YYL + 1.0027E-2 ) -
      cq26 * sq1 * sq35 * sq6 * ( YYL + 1.0027E-2 ) - cq5 * cq6 * sq12 * sq46 * ( YYL + 1.0027E-2 ) -
      cq23 * cq4 * cq6 * sq15 * ( YZL - 7.41E-4 ) - cq23 * cq6 * sq14 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq26 * sq1 * sq34 * sq5 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq4 * cq6 * sq12 * sq35 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq23 * cq45 * cq6 * sq16 * ( YYL + 1.0027E-2 ) - cq23 * cq45 * cq6 * sq16 * ( XXL + 1.2516E-2 );
  B[0][6] =
      ( sq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) +
        cq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) *
          ( XZL - 1.196E-3 ) +
      ( cq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                        cq2 * sq1 * sq3 * 0.0825 ) ) -
        sq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                        sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) ) *
          ( clx * mL + 7.735484874E-3 ) +
      ( YZL - 7.41E-4 ) *
          ( cq6 * ( sq4 * ( cq1 * sq3 - cq23 * sq1 ) - cq4 * sq12 ) -
            sq6 * ( cq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) + sq5 * ( cq13 + cq2 * sq1 * sq3 ) ) ) +
      ( ZZL + 4.815E-3 ) *
          ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) - cq5 * ( cq13 + cq2 * sq1 * sq3 ) ) -
      ( cly * mL - 3.127439544E-3 ) *
          ( sq6 * ( sq5 * ( cq4 * ( cq1 * sq3 - cq23 * sq1 ) + sq12 * sq4 ) * 0.088 -
                    cq5 * ( cq13 + cq2 * sq1 * sq3 ) * 0.088 +
                    sq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) -
                    cq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 + cq23 * sq1 * 0.384 -
                            cq2 * sq1 * sq3 * 0.0825 ) ) +
            cq6 * ( cq5 * ( cq4 * ( sq12 * 0.384 + sq12 * ( cq3 * 0.316 - sq3 * 0.0825 ) ) +
                            sq4 * ( cq1 * 0.0825 - cq13 * 0.0825 + cq2 * sq1 * 0.316 - cq1 * sq3 * 0.384 +
                                    cq23 * sq1 * 0.384 - cq2 * sq1 * sq3 * 0.0825 ) ) +
                    sq5 * ( sq12 * 0.0825 - sq12 * ( cq3 * 0.0825 + sq3 * 0.316 ) ) ) );
  B[1][1] =
      sin( q[2] * 2.0 ) * 8.126420719725001E-3 + cq3 * 2.06994360053096E-1 - sq3 * 1.205656143588E-1 +
      sq4 * 2.891981224275E-3 -
      ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) *
          ( cq24 * ( -1.158E-3 ) + sq23 * sq5 * 1.964E-3 - cq25 * sq4 * 1.964E-3 + cq3 * sq24 * 1.158E-3 +
            cq5 * sq23 * 1.09E-4 + cq2 * sq45 * 1.09E-4 - cq34 * cq5 * sq2 * 1.964E-3 + cq34 * sq25 * 1.09E-4 ) +
      cq3 * sq3 * 7.77507825633E-3 +
      pow( sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
               cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ),
           2.0 ) *
          1.666555 +
      pow( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
               cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ),
           2.0 ) *
          1.666555 +
      cq2 * ( cq2 * 3.6155E-2 - sq2 * 4.761E-3 ) -
      ( ( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                          sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                  cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
          cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                  cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                  cq34 * sq25 * 0.088 ) ) *
            ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) * 2.0 -
        ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
            ( cq24 * 0.088 +
              sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
              cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) *
            2.0 ) *
          ( clx * mL + 7.735484874E-3 ) -
      sq34 * 1.107716444692E-2 + pcq2 * 2.86206025867264E-1 - pcq3 * 3.6189455156736E-2 + psq2 * 6.80668324144E-1 +
      ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) *
          ( cq24 * 3.41E-4 + sq23 * sq5 * 1.09E-4 - cq25 * sq4 * 1.09E-4 - cq3 * sq24 * 3.41E-4 +
            cq5 * sq23 * 4.354E-3 + cq2 * sq45 * 4.354E-3 - cq34 * cq5 * sq2 * 1.09E-4 + cq34 * sq25 * 4.354E-3 ) -
      ( ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
            ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                      cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                      cq34 * sq25 * 0.088 ) +
              cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                              sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                      cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
            2.0 -
        ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
            ( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                              sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                      cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
              cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                      cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                      cq34 * sq25 * 0.088 ) ) *
            2.0 ) *
          ( clz * mL + 4.5305948634E-2 ) -
      pcq2 * cq3 * 3.147678222975E-2 - pcq2 * sq3 * 5.4041249064495E-2 - pcq3 * sq4 * 2.891981224275E-3 -
      sq2 * ( cq2 * 4.761E-3 - sq2 * 3.7242E-2 ) -
      ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
          ( ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XYL - 4.28E-4 ) -
            ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( YYL + 1.0027E-2 ) +
            ( YZL - 7.41E-4 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) ) +
      ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) *
          ( ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XZL - 1.196E-3 ) +
            ( ZZL + 4.815E-3 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
            ( YZL - 7.41E-4 ) *
                ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) ) +
      ( cq24 - cq3 * sq24 ) *
          ( sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
            cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          2.0048323339E-1 -
      ( cq24 - cq3 * sq24 ) *
          ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
            cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          4.705351387E-2 +
      ( mL + 7.35522E-1 ) * pow( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                                         cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 +
                                         cq2 * sq45 * 0.088 + cq34 * sq25 * 0.088 ) +
                                     cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                                     sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                                             cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ),
                                 2.0 ) +
      ( mL + 7.35522E-1 ) * pow( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                                 sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                                         cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
                                     cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                                             cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 +
                                             cq2 * sq45 * 0.088 + cq34 * sq25 * 0.088 ),
                                 2.0 ) +
      ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
          ( ( XZL - 1.196E-3 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
            ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XYL - 4.28E-4 ) +
            ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XXL + 1.2516E-2 ) ) -
      ( ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                  cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                  cq34 * sq25 * 0.088 ) +
          cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                          sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                  cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
            ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) * 2.0 -
        ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
            ( cq24 * 0.088 +
              sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
              cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) *
            2.0 ) *
          ( cly * mL - 3.127439544E-3 ) +
      pow( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ),
           2.0 ) *
          2.892501 +
      pow( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ), 2.0 ) *
          1.225946 +
      ( cq2 * sq4 + cq34 * sq2 ) * ( cq24 * ( -2.117E-3 ) + cq2 * sq4 * 3.5549E-2 + sq23 * 4.037E-3 +
                                     cq34 * sq2 * 3.5549E-2 + cq3 * sq24 * 2.117E-3 ) +
      ( mL + 7.35522E-1 ) * pow( cq24 * 0.088 +
                                     sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                             sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
                                     cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088,
                                 2.0 ) -
      ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) * 4.705351387E-2 +
      ( cq24 - cq3 * sq24 ) *
          ( cq24 * 5.433E-3 - sq23 * sq5 * 1.158E-3 + cq25 * sq4 * 1.158E-3 - cq3 * sq24 * 5.433E-3 +
            cq5 * sq23 * 3.41E-4 + cq2 * sq45 * 3.41E-4 + cq34 * cq5 * sq2 * 1.158E-3 + cq34 * sq25 * 3.41E-4 ) +
      pcq2 * pcq3 * 3.6189455156736E-2 +
      ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) * 2.0048323339E-1 +
      pcq2 * pow( cq3 * 0.316 - sq3 * 0.0825, 2.0 ) * 3.587895 +
      pcq2 * pow( cq3 * 0.0825 + sq3 * 0.316, 2.0 ) * 3.587895 +
      pcq2 * pow( cq3 * 0.0825 + sq3 * 0.316 - 0.0825, 2.0 ) * 1.225946 -
      ( cq24 - cq3 * sq24 ) * ( cq24 * ( -2.9474E-2 ) + cq2 * sq4 * 2.117E-3 + sq23 * 2.29E-4 + cq34 * sq2 * 2.117E-3 +
                                cq3 * sq24 * 2.9474E-2 ) +
      cq2 * ( cq2 * 2.8323E-2 + cq3 * sq2 * 1.332E-3 + sq23 * 8.640999999999999E-3 ) +
      cq3 * sq2 * ( cq2 * 1.332E-3 + cq3 * sq2 * 2.5853E-2 - sq23 * 7.796E-3 ) - pcq2 * cq3 * sq3 * 7.77507825633E-3 -
      pcq2 * cq3 * sq4 * 2.891981224275E-3 -
      sq23 *
          ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          2.9307465076E-2 +
      sq23 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          1.0068694498E-1 -
      cq2 * ( cq2 * sq4 + cq34 * sq2 ) * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 1.0068694498E-1 -
      cq2 * ( cq24 - cq3 * sq24 ) * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 2.9307465076E-2 +
      sq23 * ( cq2 * 8.640999999999999E-3 - cq3 * sq2 * 7.796E-3 + sq23 * 1.9552E-2 ) +
      pcq2 * pcq3 * sq4 * 2.891981224275E-3 +
      sq23 * ( cq24 * ( -2.29E-4 ) + cq2 * sq4 * 4.037E-3 + sq23 * 8.626999999999999E-3 + cq34 * sq2 * 4.037E-3 +
               cq3 * sq24 * 2.29E-4 ) -
      cq24 * sq2 * 2.891981224275E-3 - cq3 * sq34 * 1.346085806208E-2 + pcq2 * cq3 * sq34 * 1.346085806208E-2 +
      cq23 * cq4 * sq2 * 2.891981224275E-3 + cq24 * sq23 * 1.346085806208E-2 - 1.09700849867456E-1;
  B[1][2] =
      cq2 * ( -2.1724761981448E-2 ) - sq2 * 1.33007355709552E-1 -
      ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq3 * sq5 + cq45 * sq3 ) * 2.3526756935E-2 -
      ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq35 - cq4 * sq35 ) * 1.00241616695E-1 -
      cq24 * 5.538582223460001E-3 + cq2 * sq4 * 4.630579482008E-3 - cq4 * sq2 * 5.627033294592E-3 +
      sq2 * ( cq3 * 0.316 + sq3 * 0.0825 ) * 1.9076837715E-1 - sq2 * ( cq3 * 0.0825 - sq3 * 0.316 ) * 3.74644408005E-1 -
      sq24 * 6.730429031040001E-3 +
      ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
          ( ( YYL + 1.0027E-2 ) * ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) +
            ( YZL - 7.41E-4 ) * ( cq35 - cq4 * sq35 ) -
            ( XYL - 4.28E-4 ) * ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) ) *
          0.5 -
      ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) *
          ( ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XYL - 4.28E-4 ) -
            ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( YYL + 1.0027E-2 ) +
            ( YZL - 7.41E-4 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) ) *
          0.5 +
      ( cq24 - cq3 * sq24 ) *
          ( cq35 * ( -3.41E-4 ) + cq3 * sq5 * 1.158E-3 - sq34 * 5.433E-3 + cq45 * sq3 * 1.158E-3 +
            cq4 * sq35 * 3.41E-4 ) *
          0.5 +
      ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) *
          ( ( XZL - 1.196E-3 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
            ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XYL - 4.28E-4 ) +
            ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XXL + 1.2516E-2 ) ) *
          0.5 -
      ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) *
          ( ( YZL - 7.41E-4 ) * ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) +
            ( ZZL + 4.815E-3 ) * ( cq35 - cq4 * sq35 ) -
            ( XZL - 1.196E-3 ) * ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) ) *
          0.5 -
      ( cq24 - cq3 * sq24 ) * ( cq3 * ( -2.29E-4 ) + cq4 * sq3 * 2.117E-3 + sq34 * 2.9474E-2 ) * 0.5 +
      ( clx * mL + 7.735484874E-3 ) *
          ( ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) *
                ( cq24 * 0.088 +
                  sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                          sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
                  cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) +
            ( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
              cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) *
                ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) +
            ( cq35 - cq4 * sq35 ) * ( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                                      sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                                              cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
                                      cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                                              cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 +
                                              cq2 * sq45 * 0.088 + cq34 * sq25 * 0.088 ) ) -
            sq4 * ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) *
                ( sq3 * 0.088 - sq5 * 0.0825 + cq3 * sq5 * 0.0825 + sq35 * 0.384 ) ) -
      ( cq3 * sq5 + cq45 * sq3 ) *
          ( cq24 * ( -1.158E-3 ) + sq23 * sq5 * 1.964E-3 - cq25 * sq4 * 1.964E-3 + cq3 * sq24 * 1.158E-3 +
            cq5 * sq23 * 1.09E-4 + cq2 * sq45 * 1.09E-4 - cq34 * cq5 * sq2 * 1.964E-3 + cq34 * sq25 * 1.09E-4 ) *
          0.5 +
      sq3 * ( cq2 * 1.332E-3 + cq3 * sq2 * 2.5853E-2 - sq23 * 7.796E-3 ) * 0.5 -
      ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
          ( ( XYL - 4.28E-4 ) * ( -cq6 * sq34 + cq3 * sq56 + cq45 * sq36 ) -
            ( XXL + 1.2516E-2 ) * ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) +
            ( cq35 - cq4 * sq35 ) * ( XZL - 1.196E-3 ) ) *
          0.5 -
      ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) *
          ( cq35 * 4.354E-3 + cq3 * sq5 * 1.09E-4 + sq34 * 3.41E-4 + cq45 * sq3 * 1.09E-4 - cq4 * sq35 * 4.354E-3 ) *
          0.5 -
      cq3 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          5.034347249E-2 -
      ( clz * mL + 4.5305948634E-2 ) *
          ( cq4 * sq2 * ( -0.088 ) - cq5 * sq2 * 0.0825 + pcq3 * cq4 * sq2 * 0.176 + pcq3 * cq5 * sq2 * 0.165 +
            cq4 * pcq5 * sq2 * 0.176 + cq23 * sq4 * 0.088 - cq35 * sq2 * 0.0825 + cq5 * sq23 * 0.316 +
            cq2 * sq45 * 0.316 - cq4 * sq25 * 0.384 - cq23 * pcq5 * sq4 * 0.176 + pcq3 * cq4 * sq25 * 0.768 -
            pcq3 * cq4 * pcq5 * sq2 * 0.352 + cq35 * sq23 * 0.768 + cq23 * sq45 * 0.384 + cq34 * sq25 * 0.316 -
            cq2 * sq34 * sq5 * 0.0825 + cq4 * sq23 * sq5 * 0.0825 - cq34 * sq23 * sq5 * 0.165 +
            cq35 * sq23 * sq5 * 0.176 + cq24 * cq5 * sq34 * sq5 * 0.176 + cq3 * pcq4 * cq5 * sq23 * sq5 * 0.176 ) +
      ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) *
          ( cq35 * 1.09E-4 + cq3 * sq5 * 1.964E-3 - sq34 * 1.158E-3 + cq45 * sq3 * 1.964E-3 - cq4 * sq35 * 1.09E-4 ) *
          0.5 -
      cq3 * ( cq2 * 8.640999999999999E-3 - cq3 * sq2 * 7.796E-3 + sq23 * 1.9552E-2 ) * 0.5 -
      ( cq35 - cq4 * sq35 ) *
          ( cq24 * 3.41E-4 + sq23 * sq5 * 1.09E-4 - cq25 * sq4 * 1.09E-4 - cq3 * sq24 * 3.41E-4 +
            cq5 * sq23 * 4.354E-3 + cq2 * sq45 * 4.354E-3 - cq34 * cq5 * sq2 * 1.09E-4 + cq34 * sq25 * 4.354E-3 ) *
          0.5 +
      ( cq2 * sq4 + cq34 * sq2 ) * ( cq3 * ( -4.037E-3 ) + cq4 * sq3 * 3.5549E-2 + sq34 * 2.117E-3 ) * 0.5 +
      ( cly * mL - 3.127439544E-3 ) *
          ( ( sq34 * sq6 + cq36 * sq5 + cq45 * cq6 * sq3 ) *
                ( cq24 * 0.088 +
                  sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                          sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
                  cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) -
            ( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
              cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) *
                ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) +
            ( cq35 - cq4 * sq35 ) * ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                                              cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 +
                                              cq2 * sq45 * 0.088 + cq34 * sq25 * 0.088 ) +
                                      cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                                      sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                                              cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) -
            sq4 * ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( sq3 * 0.088 - sq5 * 0.0825 + cq3 * sq5 * 0.0825 + sq35 * 0.384 ) ) -
      ( cq35 - cq4 * sq35 ) *
          ( ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) *
                ( XZL - 1.196E-3 ) +
            ( ZZL + 4.815E-3 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
            ( YZL - 7.41E-4 ) *
                ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) ) ) *
          0.5 -
      sq2 * ( cq3 * ( -0.0825 ) + sq3 * 0.316 + cq3 * sq3 * 0.768 + pcq3 * 0.165 - 0.0825 ) * 4.7121686402E-2 -
      cq2 * ( cq3 * 8.640999999999999E-3 - sq3 * 1.332E-3 ) * 0.5 -
      cq3 *
          ( cq24 * ( -2.29E-4 ) + cq2 * sq4 * 4.037E-3 + sq23 * 8.626999999999999E-3 + cq34 * sq2 * 4.037E-3 +
            cq3 * sq24 * 2.29E-4 ) *
          0.5 +
      sq23 * sq4 * 1.4459906121375E-3 +
      sq23 * ( cq3 * ( -8.626999999999999E-3 ) + cq4 * sq3 * 4.037E-3 + sq34 * 2.29E-4 ) * 0.5 +
      cq3 * sq2 * ( cq3 * 7.796E-3 + sq3 * 2.5853E-2 ) * 0.5 -
      cq4 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) *
          2.3526756935E-2 -
      sq34 *
          ( cq24 * 5.433E-3 - sq23 * sq5 * 1.158E-3 + cq25 * sq4 * 1.158E-3 - cq3 * sq24 * 5.433E-3 +
            cq5 * sq23 * 3.41E-4 + cq2 * sq45 * 3.41E-4 + cq34 * cq5 * sq2 * 1.158E-3 + cq34 * sq25 * 3.41E-4 ) *
          0.5 +
      sq34 *
          ( cq24 * ( -2.9474E-2 ) + cq2 * sq4 * 2.117E-3 + sq23 * 2.29E-4 + cq34 * sq2 * 2.117E-3 +
            cq3 * sq24 * 2.9474E-2 ) *
          0.5 +
      cq4 * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.00241616695E-1 +
      cq4 *
          ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 2.892501 +
      pcq3 * cq4 * sq2 * 1.1254066589184E-2 -
      sq4 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.225946 +
      pcq3 * sq24 * 1.346085806208E-2 - sq23 * ( cq3 * 1.9552E-2 + sq3 * 7.796E-3 ) * 0.5 -
      cq23 * cq4 * 6.730429031040001E-3 -
      sq34 *
          ( sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
            cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          1.00241616695E-1 +
      sq34 *
          ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
            cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          2.3526756935E-2 +
      cq23 * sq4 * 5.627033294592E-3 + cq24 * sq3 * 1.4459906121375E-3 + cq34 * sq2 * 4.630579482008E-3 +
      cq4 * sq3 *
          ( cq24 * ( -2.117E-3 ) + cq2 * sq4 * 3.5549E-2 + sq23 * 4.037E-3 + cq34 * sq2 * 3.5549E-2 +
            cq3 * sq24 * 2.117E-3 ) *
          0.5 -
      cq2 * sq34 * 1.208932934385E-3 + cq3 * sq24 * 5.538582223460001E-3 + cq4 * sq23 * 1.208932934385E-3 +
      ( mL + 7.35522E-1 ) *
          ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                    cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                    cq34 * sq25 * 0.088 ) +
            cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                            sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                    cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) *
          ( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
            cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) -
      ( mL + 7.35522E-1 ) *
          ( sq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                            sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                    cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
            cq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                    cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) + cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 +
                    cq34 * sq25 * 0.088 ) ) *
          ( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
            cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) +
      cq5 * sq4 * ( cq24 - cq3 * sq24 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 2.3526756935E-2 -
      sq4 * ( mL + 7.35522E-1 ) * ( sq3 * 0.088 - sq5 * 0.0825 + cq3 * sq5 * 0.0825 + sq35 * 0.384 ) *
          ( cq24 * 0.088 +
            sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
            cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) - cq3 * sq24 * 0.088 ) -
      sq45 * ( cq24 - cq3 * sq24 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.00241616695E-1 -
      cq5 * sq4 *
          ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
            cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.666555 -
      sq45 *
          ( sq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) +
            cq25 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) *
          ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.666555 -
      cq24 * sq3 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) * 5.034347249E-2 - cq34 * sq23 * 2.41786586877E-3 -
      cq3 * sq23 * sq4 * 2.891981224275E-3 - sq23 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 5.034347249E-2;
  B[1][3] =
      cq2 * ( -4.78240257230514E-1 ) - cq23 * 4.19059345324423E-1 - cq24 * 2.41786586877E-3 + cq25 * 1.806854932608E-2 +
      cq2 * sq3 * 2.543453644616475E-1 - cq3 * sq2 * 3.449E-3 - cq2 * sq4 * 1.1198654185125E-2 -
      cq2 * sq5 * 7.698556162176E-2 - sq23 * 8.640999999999999E-3 - cq2 * ( ZZL + 4.815E-3 ) -
      cq2 * ( mL + 7.35522E-1 ) * 1.6200625E-1 + cq2 * pcq4 * 4.995999999999997E-3 + cq2 * pcq5 * 2.39E-3 +
      sq23 * sq4 * 1.481033294592E-3 - cq2 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) * 2.0 -
      cq23 * ( mL * 1.21344E-1 + 8.9251181568E-2 ) - cq2 * pcq5 * ( YYL + 1.0027E-2 ) + cq3 * pcq4 * sq2 * 4.234E-3 +
      cq2 * pcq4 * ( ZZL + 4.815E-3 ) + cq2 * pcq5 * ( ZZL + 4.815E-3 ) +
      cq25 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) * 2.0 + cq2 * sq3 * ( mL * 2.607E-2 + 1.917505854E-2 ) +
      cq23 * ( mL * 6.806250000000001E-3 + 5.006146612500001E-3 ) + cq2 * pcq5 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      cq2 * pcq4 * pcq5 * 2.39E-3 + cq2 * sq3 * ( mL * 3.168E-2 + 2.330133696E-2 ) -
      cq26 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 + cq23 * cq4 * 1.208932934385E-3 +
      cq23 * cq5 * 7.434455191459999E-3 + cq24 * cq5 * 1.6539866754675E-2 +
      cq2 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 + cq23 * sq4 * 5.5993270925625E-3 +
      cq24 * sq3 * 4.630579482008E-3 - cq34 * sq2 * 5.5993270925625E-3 - cq23 * sq5 * 3.167635087562E-2 +
      cq24 * sq4 * 4.234E-3 - cq25 * sq3 * 1.9409574471375E-3 + cq35 * sq2 * 1.158E-3 + cq24 * sq5 * 3.881914894275E-3 +
      cq25 * sq5 * 2.18E-4 - cq2 * pcq4 * ( XXL + 1.2516E-2 ) + cq2 * sq34 * 2.14471195303E-2 +
      cq3 * sq24 * 1.208932934385E-3 - cq4 * sq23 * 2.58333224672E-2 + cq2 * sq35 * 8.269933377337502E-3 +
      cq3 * sq25 * 3.41E-4 + cq23 * cq5 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) +
      cq2 * pcq5 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq26 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) * 2.0 +
      cq24 * cq5 * ( mL * 7.26E-3 + 5.33988972E-3 ) * 2.0 -
      cq2 * pcq5 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq3 * pcq4 * cq5 * sq2 * 2.316E-3 -
      cq2 * pcq4 * cq5 * sq5 * 2.18E-4 + cq2 * pcq4 * pcq6 * ( XXL + 1.2516E-2 ) -
      cq2 * pcq5 * pcq6 * ( XXL + 1.2516E-2 ) - cq3 * pcq4 * sq25 * 6.82E-4 +
      cq2 * sq35 * ( mL * 7.26E-3 + 5.33988972E-3 ) + cq25 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      pcq5 * sq23 * sq4 * 2.18E-4 + cq26 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) * 2.0 +
      cq2 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) * 2.0 -
      sq23 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) + cq35 * sq2 * ( XYL - 4.28E-4 ) +
      cq2 * pcq4 * pcq5 * ( YYL + 1.0027E-2 ) - cq2 * pcq4 * pcq6 * ( YYL + 1.0027E-2 ) +
      cq2 * pcq5 * pcq6 * ( YYL + 1.0027E-2 ) - cq23 * sq5 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) +
      cq2 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) * 2.0 - cq2 * pcq4 * pcq5 * ( ZZL + 4.815E-3 ) -
      cq23 * cq45 * 8.269933377337502E-3 - cq23 * cq4 * sq5 * 1.9409574471375E-3 -
      cq24 * cq5 * sq3 * 3.167635087562E-2 - cq24 * cq5 * sq4 * 2.316E-3 -
      cq2 * pcq4 * pcq5 * ( mL + 7.35522E-1 ) * 7.744E-3 - cq25 * sq3 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq24 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) * 2.0 - cq34 * sq24 * 4.995999999999997E-3 -
      cq24 * sq35 * 7.434455191459999E-3 - cq35 * sq24 * 8.269933377337502E-3 - cq45 * sq23 * 3.41E-4 -
      cq24 * sq45 * 6.82E-4 - cq3 * sq24 * sq5 * 1.9409574471375E-3 + cq4 * sq23 * sq5 * 1.158E-3 -
      cq5 * sq23 * sq4 * 3.849278081088E-2 - sq23 * sq45 * 9.03427466304E-3 +
      cq24 * cq5 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) * 2.0 + cq34 * pcq5 * sq24 * 2.39E-3 -
      sq23 * sq46 * ( YZL - 7.41E-4 ) + cq4 * sq23 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      cq35 * sq24 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq23 * cq6 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq34 * cq6 * sq2 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq24 * cq5 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) * 2.0 -
      cq23 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq34 * sq26 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq26 * sq35 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) - cq24 * cq5 * sq4 * ( XYL - 4.28E-4 ) * 2.0 -
      cq2 * sq35 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq2 * pcq4 * pcq5 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq25 * cq6 * sq5 * ( XZL - 1.196E-3 ) * 2.0 - cq24 * cq5 * sq3 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) +
      cq2 * pcq4 * pcq5 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq2 * pcq4 * pcq5 * pcq6 * ( XXL + 1.2516E-2 ) - sq23 * sq45 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) +
      cq4 * sq23 * sq5 * ( XYL - 4.28E-4 ) + cq34 * sq24 * ( XXL + 1.2516E-2 ) +
      cq4 * cq6 * sq23 * ( cly * mL * 0.384 - 1.200936784896E-3 ) + cq6 * sq23 * sq4 * ( XZL - 1.196E-3 ) -
      cq3 * sq25 * sq6 * ( XZL - 1.196E-3 ) - cq2 * pcq4 * cq5 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      cq23 * cq6 * sq5 * ( clx * mL * 0.316 + 2.444413220184E-3 ) -
      cq26 * sq34 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) +
      cq23 * sq56 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) +
      cq35 * cq6 * sq2 * ( cly * mL * 0.088 - 2.75214679872E-4 ) +
      pcq5 * sq23 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      cq3 * pcq4 * cq5 * sq2 * ( XYL - 4.28E-4 ) * 2.0 - cq35 * pcq6 * sq2 * ( XYL - 4.28E-4 ) * 2.0 -
      cq2 * pcq4 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + cq2 * pcq5 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq23 * cq4 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      cq2 * sq34 * sq6 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq35 * sq26 * ( clx * mL * 0.088 + 6.80722668912E-4 ) - cq2 * pcq4 * pcq5 * pcq6 * ( YYL + 1.0027E-2 ) -
      cq3 * sq24 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) - cq5 * sq23 * sq45 * 2.39E-3 -
      cq34 * sq24 * ( ZZL + 4.815E-3 ) - cq5 * sq23 * sq4 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) -
      cq36 * sq25 * ( YZL - 7.41E-4 ) + cq25 * sq56 * ( YZL - 7.41E-4 ) * 2.0 -
      cq23 * cq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq24 * sq35 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) +
      cq34 * pcq5 * sq24 * ( ZZL + 4.815E-3 ) + cq35 * sq24 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq3 * pcq4 * cq6 * sq25 * ( YZL - 7.41E-4 ) * 2.0 - cq2 * pcq4 * cq5 * sq56 * ( YZL - 7.41E-4 ) * 2.0 +
      cq34 * pcq5 * sq24 * ( mL + 7.35522E-1 ) * 7.744E-3 - cq35 * cq6 * sq26 * ( XXL + 1.2516E-2 ) +
      cq45 * sq23 * sq6 * ( XZL - 1.196E-3 ) + cq24 * sq45 * sq6 * ( XZL - 1.196E-3 ) * 2.0 +
      pcq5 * sq23 * sq46 * ( YZL - 7.41E-4 ) * 2.0 + cq5 * sq23 * sq46 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq24 * cq5 * cq6 * sq3 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq24 * cq5 * sq36 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) -
      cq24 * cq5 * cq6 * sq4 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq24 * cq5 * pcq6 * sq4 * ( XYL - 4.28E-4 ) * 4.0 -
      cq24 * cq5 * sq46 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq2 * pcq4 * cq5 * cq6 * sq5 * ( XZL - 1.196E-3 ) * 2.0 + cq35 * cq6 * sq26 * ( YYL + 1.0027E-2 ) +
      cq4 * cq6 * sq23 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + cq34 * cq5 * sq24 * sq5 * 2.18E-4 -
      cq4 * pcq6 * sq23 * sq5 * ( XYL - 4.28E-4 ) * 2.0 - cq34 * pcq6 * sq24 * ( XXL + 1.2516E-2 ) +
      cq4 * sq23 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + cq3 * pcq4 * sq25 * sq6 * ( XZL - 1.196E-3 ) * 2.0 -
      pcq5 * cq6 * sq23 * sq4 * ( XZL - 1.196E-3 ) * 2.0 + cq5 * sq23 * sq45 * ( YYL + 1.0027E-2 ) +
      cq45 * cq6 * sq23 * ( YZL - 7.41E-4 ) + cq24 * cq6 * sq45 * ( YZL - 7.41E-4 ) * 2.0 -
      cq3 * pcq4 * cq5 * cq6 * sq2 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq5 * sq23 * sq45 * ( ZZL + 4.815E-3 ) + cq3 * pcq4 * cq5 * pcq6 * sq2 * ( XYL - 4.28E-4 ) * 4.0 -
      cq2 * pcq4 * pcq5 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq3 * pcq4 * cq5 * sq26 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq23 * cq45 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      cq5 * cq6 * sq23 * sq4 * ( clx * mL * 0.384 + 2.970426191616E-3 ) - cq34 * pcq5 * sq24 * ( YYL + 1.0027E-2 ) +
      cq34 * pcq6 * sq24 * ( YYL + 1.0027E-2 ) + cq23 * cq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq5 * sq23 * sq45 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      cq35 * cq6 * sq24 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq34 * cq6 * sq24 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
      cq24 * cq5 * cq6 * sq46 * ( XXL + 1.2516E-2 ) * 2.0 + cq34 * pcq5 * pcq6 * sq24 * ( YYL + 1.0027E-2 ) -
      cq4 * cq6 * sq23 * sq56 * ( XXL + 1.2516E-2 ) - cq24 * cq5 * cq6 * sq46 * ( YYL + 1.0027E-2 ) * 2.0 +
      cq3 * pcq4 * cq5 * cq6 * sq26 * ( XXL + 1.2516E-2 ) * 2.0 -
      cq5 * cq6 * sq23 * sq45 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq4 * cq6 * sq23 * sq56 * ( YYL + 1.0027E-2 ) +
      cq5 * sq23 * sq45 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq5 * pcq6 * sq23 * sq45 * ( XXL + 1.2516E-2 ) +
      cq34 * pcq5 * cq6 * sq24 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq3 * pcq4 * cq5 * cq6 * sq26 * ( YYL + 1.0027E-2 ) * 2.0 -
      cq34 * pcq5 * sq24 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq34 * pcq5 * pcq6 * sq24 * ( XXL + 1.2516E-2 ) - cq5 * pcq6 * sq23 * sq45 * ( YYL + 1.0027E-2 ) +
      cq34 * cq5 * sq24 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq34 * pcq5 * cq6 * sq24 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + cq34 * cq5 * sq24 * sq56 * ( YZL - 7.41E-4 ) * 2.0 -
      cq34 * cq5 * cq6 * sq24 * sq5 * ( XZL - 1.196E-3 ) * 2.0 -
      cq5 * cq6 * sq23 * sq45 * sq6 * ( XYL - 4.28E-4 ) * 2.0;
  B[1][4] = cq24 * ( -2.58333224672E-2 ) + cq2 * sq4 * 1.481033294592E-3 + cq4 * sq2 * 4.630579482008E-3 -
            sq23 * 1.0591E-2 + sq24 * 2.14471195303E-2 - sq23 * ( YYL + 1.0027E-2 ) - sq23 * sq4 * 5.5993270925625E-3 +
            cq2 * pcq5 * sq4 * 2.18E-4 - pcq5 * sq23 * 2.39E-3 - cq2 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) -
            cq23 * cq4 * 2.14471195303E-2 - cq24 * cq5 * 3.41E-4 + cq23 * sq4 * 4.630579482008E-3 +
            cq24 * sq3 * 5.5993270925625E-3 + cq34 * sq2 * 1.481033294592E-3 + cq24 * sq5 * 1.158E-3 -
            cq25 * sq4 * 3.849278081088E-2 - cq45 * sq2 * 3.167635087562E-2 - cq2 * sq34 * 1.208932934385E-3 +
            cq3 * sq24 * 2.58333224672E-2 - cq4 * sq23 * 1.208932934385E-3 - cq2 * sq45 * 9.03427466304E-3 -
            cq4 * sq25 * 7.434455191459999E-3 - cq2 * sq46 * ( YZL - 7.41E-4 ) +
            cq24 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
            cq4 * sq25 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) + cq34 * pcq5 * sq2 * 2.18E-4 +
            pcq5 * sq23 * ( YYL + 1.0027E-2 ) + pcq6 * sq23 * ( YYL + 1.0027E-2 ) -
            cq34 * sq2 * ( clz * mL * 0.088 + 3.986923479792E-3 ) - pcq5 * sq23 * ( ZZL + 4.815E-3 ) -
            cq2 * sq45 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) + cq24 * sq5 * ( XYL - 4.28E-4 ) +
            cq24 * cq6 * ( cly * mL * 0.384 - 1.200936784896E-3 ) + cq26 * sq4 * ( XZL - 1.196E-3 ) -
            cq45 * sq2 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) - pcq5 * sq23 * ( mL + 7.35522E-1 ) * 7.744E-3 +
            cq2 * pcq5 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
            cq6 * sq24 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) - cq23 * cq5 * sq4 * 3.167635087562E-2 -
            cq34 * cq5 * sq2 * 3.849278081088E-2 - sq24 * sq6 * ( clx * mL * 0.316 + 2.444413220184E-3 ) -
            cq23 * sq45 * 7.434455191459999E-3 + cq25 * sq34 * 8.269933377337502E-3 - cq34 * sq25 * 9.03427466304E-3 +
            cq35 * sq24 * 3.41E-4 + cq45 * sq23 * 8.269933377337502E-3 - cq25 * sq45 * 2.39E-3 +
            cq2 * sq34 * sq5 * 1.9409574471375E-3 - cq3 * sq24 * sq5 * 1.158E-3 +
            cq4 * sq23 * sq5 * 1.9409574471375E-3 - cq5 * sq23 * sq5 * 2.18E-4 - pcq6 * sq23 * ( XXL + 1.2516E-2 ) -
            cq25 * sq4 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) -
            pcq5 * cq6 * sq23 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
            cq3 * sq24 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
            cq25 * sq34 * ( mL * 7.26E-3 + 5.33988972E-3 ) + cq45 * sq23 * ( mL * 7.26E-3 + 5.33988972E-3 ) +
            pcq5 * sq23 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
            pcq5 * pcq6 * sq23 * ( XXL + 1.2516E-2 ) - cq24 * cq6 * sq3 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
            cq24 * sq36 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
            cq5 * sq23 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
            cq34 * sq25 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) +
            cq6 * sq23 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
            sq23 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq34 * cq6 * sq2 * ( XZL - 1.196E-3 ) +
            cq24 * cq5 * sq6 * ( XZL - 1.196E-3 ) + cq2 * pcq5 * sq46 * ( YZL - 7.41E-4 ) * 2.0 -
            cq23 * cq5 * sq4 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) - cq3 * sq24 * sq5 * ( XYL - 4.28E-4 ) +
            cq6 * sq23 * sq6 * ( XYL - 4.28E-4 ) * 2.0 - cq36 * sq24 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
            cq25 * sq46 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
            cq23 * cq4 * cq6 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) - pcq5 * pcq6 * sq23 * ( YYL + 1.0027E-2 ) +
            cq34 * pcq5 * sq2 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
            cq23 * cq4 * sq6 * ( clx * mL * 0.316 + 2.444413220184E-3 ) -
            cq45 * cq6 * sq2 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
            cq45 * sq26 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) +
            cq24 * cq6 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) - cq24 * pcq6 * sq5 * ( XYL - 4.28E-4 ) * 2.0 +
            cq24 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) - cq34 * cq5 * sq25 * 2.39E-3 -
            cq2 * pcq5 * cq6 * sq4 * ( XZL - 1.196E-3 ) * 2.0 + cq25 * sq45 * ( YYL + 1.0027E-2 ) +
            cq2 * sq34 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
            cq4 * sq23 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
            cq34 * cq5 * sq2 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) + cq24 * cq5 * cq6 * ( YZL - 7.41E-4 ) -
            cq25 * sq45 * ( ZZL + 4.815E-3 ) - cq34 * sq26 * ( YZL - 7.41E-4 ) -
            cq25 * cq6 * sq4 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
            cq23 * sq45 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) - cq25 * sq45 * ( mL + 7.35522E-1 ) * 7.744E-3 -
            cq25 * sq34 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
            cq45 * sq23 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) + cq34 * pcq5 * sq26 * ( YZL - 7.41E-4 ) * 2.0 -
            cq24 * cq6 * sq56 * ( XXL + 1.2516E-2 ) + cq34 * cq5 * sq26 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
            cq35 * sq24 * sq6 * ( XZL - 1.196E-3 ) + cq5 * cq6 * sq23 * sq5 * ( XZL - 1.196E-3 ) * 2.0 -
            cq23 * cq5 * cq6 * sq4 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
            cq23 * cq5 * sq46 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) -
            cq25 * cq6 * sq45 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
            cq34 * pcq5 * cq6 * sq2 * ( XZL - 1.196E-3 ) * 2.0 + cq34 * cq5 * sq25 * ( YYL + 1.0027E-2 ) +
            cq24 * cq6 * sq56 * ( YYL + 1.0027E-2 ) - cq36 * sq24 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) +
            cq25 * sq45 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
            cq3 * pcq6 * sq24 * sq5 * ( XYL - 4.28E-4 ) * 2.0 - pcq5 * cq6 * sq23 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
            cq25 * pcq6 * sq45 * ( XXL + 1.2516E-2 ) - cq3 * sq24 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
            cq34 * cq5 * sq25 * ( ZZL + 4.815E-3 ) - cq35 * cq6 * sq24 * ( YZL - 7.41E-4 ) -
            cq34 * cq5 * cq6 * sq2 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
            cq34 * cq5 * sq25 * ( mL + 7.35522E-1 ) * 7.744E-3 - cq5 * sq23 * sq56 * ( YZL - 7.41E-4 ) * 2.0 -
            cq25 * pcq6 * sq45 * ( YYL + 1.0027E-2 ) + cq25 * cq6 * sq34 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
            cq45 * cq6 * sq23 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
            cq25 * cq6 * sq45 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + cq36 * sq24 * sq56 * ( XXL + 1.2516E-2 ) -
            cq34 * cq5 * cq6 * sq25 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
            cq34 * cq5 * sq25 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
            cq34 * cq5 * pcq6 * sq25 * ( XXL + 1.2516E-2 ) - cq36 * sq24 * sq56 * ( YYL + 1.0027E-2 ) -
            cq34 * cq5 * pcq6 * sq25 * ( YYL + 1.0027E-2 ) - cq34 * cq5 * cq6 * sq25 * sq6 * ( XYL - 4.28E-4 ) * 2.0;
  B[1][5] =
      cq24 * ( -5.433E-3 ) + cq25 * 8.269933377337502E-3 + cq2 * sq5 * 1.9409574471375E-3 + sq23 * sq5 * 1.158E-3 +
      sq24 * sq5 * 3.167635087562E-2 - cq24 * ( mL + 7.35522E-1 ) * 7.744E-3 + cq25 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      cq24 * ( XXL + 1.2516E-2 ) - cq23 * cq5 * 8.269933377337502E-3 + cq24 * cq5 * 9.03427466304E-3 -
      cq23 * sq5 * 1.9409574471375E-3 - cq25 * sq3 * 3.167635087562E-2 - cq24 * sq5 * 3.849278081088E-2 -
      cq25 * sq4 * 1.158E-3 + cq2 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) + cq3 * sq24 * 5.433E-3 -
      cq2 * sq35 * 7.434455191459999E-3 - cq5 * sq23 * 3.41E-4 - cq2 * sq45 * 3.41E-4 -
      cq5 * sq24 * 7.434455191459999E-3 - cq23 * cq5 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      cq24 * pcq6 * ( YYL + 1.0027E-2 ) - cq2 * sq35 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) -
      cq5 * sq24 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) + cq3 * sq24 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq25 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq24 * cq5 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) -
      cq25 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq25 * sq4 * ( XYL - 4.28E-4 ) -
      cq25 * sq3 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) + sq23 * sq5 * ( XYL - 4.28E-4 ) +
      cq3 * sq24 * ( XXL + 1.2516E-2 ) + sq24 * sq5 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) -
      cq24 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 + cq23 * cq45 * 7.434455191459999E-3 +
      cq24 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq23 * cq4 * sq5 * 3.167635087562E-2 -
      cq24 * cq5 * sq3 * 1.9409574471375E-3 - cq34 * cq5 * sq2 * 1.158E-3 + cq24 * pcq6 * ( XXL + 1.2516E-2 ) -
      cq23 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) + cq24 * sq35 * 8.269933377337502E-3 -
      cq34 * sq25 * 3.41E-4 - cq35 * sq24 * 9.03427466304E-3 + cq3 * sq24 * sq5 * 3.849278081088E-2 +
      cq5 * sq23 * sq4 * 1.9409574471375E-3 - cq24 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) -
      sq23 * sq45 * 8.269933377337502E-3 - cq23 * cq5 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq24 * sq35 * ( mL * 7.26E-3 + 5.33988972E-3 ) + cq3 * pcq6 * sq24 * ( YYL + 1.0027E-2 ) +
      cq23 * cq5 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - sq23 * sq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) -
      cq35 * sq24 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) - cq34 * cq5 * sq2 * ( XYL - 4.28E-4 ) -
      cq24 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 - cq23 * cq4 * sq5 * ( mL * 2.7808E-2 + 2.0453395776E-2 ) +
      cq24 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) + cq5 * sq23 * sq6 * ( XZL - 1.196E-3 ) +
      cq2 * sq45 * sq6 * ( XZL - 1.196E-3 ) - cq25 * cq6 * sq3 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq25 * sq36 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) -
      cq25 * cq6 * sq4 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + cq25 * pcq6 * sq4 * ( XYL - 4.28E-4 ) * 2.0 -
      cq24 * cq5 * sq3 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq6 * sq24 * sq5 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq36 * sq24 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
      cq25 * sq46 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      sq24 * sq56 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) -
      cq3 * sq24 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      cq6 * sq23 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) - pcq6 * sq23 * sq5 * ( XYL - 4.28E-4 ) * 2.0 -
      cq3 * pcq6 * sq24 * ( XXL + 1.2516E-2 ) + cq5 * sq23 * sq4 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      sq23 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + cq23 * cq45 * ( clz * mL * 0.316 + 1.4316679768344E-2 ) +
      cq3 * sq24 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) + cq5 * cq6 * sq23 * ( YZL - 7.41E-4 ) +
      cq26 * sq45 * ( YZL - 7.41E-4 ) - cq24 * cq6 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      cq24 * sq35 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq6 * sq23 * sq45 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq36 * sq24 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
      sq23 * sq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) + cq25 * cq6 * sq46 * ( XXL + 1.2516E-2 ) +
      cq34 * sq25 * sq6 * ( XZL - 1.196E-3 ) - cq6 * sq23 * sq56 * ( XXL + 1.2516E-2 ) -
      cq3 * sq24 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq23 * cq4 * cq6 * sq5 * ( clx * mL * 0.316 + 2.444413220184E-3 ) +
      cq23 * cq4 * sq56 * ( cly * mL * 0.316 - 9.882708959039999E-4 ) -
      cq34 * cq5 * cq6 * sq2 * ( cly * mL * 0.088 - 2.75214679872E-4 ) +
      cq34 * cq5 * pcq6 * sq2 * ( XYL - 4.28E-4 ) * 2.0 - cq34 * cq5 * sq26 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
      cq25 * cq6 * sq46 * ( YYL + 1.0027E-2 ) + cq6 * sq23 * sq56 * ( YYL + 1.0027E-2 ) +
      cq34 * cq6 * sq25 * ( YZL - 7.41E-4 ) + cq36 * sq24 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq24 * cq6 * sq35 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq34 * cq5 * cq6 * sq26 * ( XXL + 1.2516E-2 ) -
      cq34 * cq5 * cq6 * sq26 * ( YYL + 1.0027E-2 );
  B[1][6] =
      -( cly * mL - 3.127439544E-3 ) * ( sq6 * ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) +
                                                 cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) +
                                                 cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 + cq34 * sq25 * 0.088 ) +
                                         cq6 * ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) -
                                                         sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
                                                 cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) ) +
      ( cq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) - sq6 * ( cq24 - cq3 * sq24 ) ) * ( XZL - 1.196E-3 ) +
      ( ZZL + 4.815E-3 ) * ( cq5 * sq23 + cq2 * sq45 + cq34 * sq25 ) -
      ( sq6 *
            ( cq5 * ( cq24 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) - sq24 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) ) -
              cq2 * sq5 * ( cq3 * 0.0825 + sq3 * 0.316 - 0.0825 ) ) -
        cq6 *
            ( cq2 * sq4 * ( cq3 * 0.316 - sq3 * 0.0825 + 0.384 ) + cq4 * sq2 * ( cq3 * 0.384 - sq3 * 0.0825 + 0.316 ) +
              cq5 * sq23 * 0.088 + cq2 * sq45 * 0.088 + cq34 * sq25 * 0.088 ) ) *
          ( clx * mL + 7.735484874E-3 ) -
      ( YZL - 7.41E-4 ) * ( sq6 * ( -sq23 * sq5 + cq25 * sq4 + cq34 * cq5 * sq2 ) + cq6 * ( cq24 - cq3 * sq24 ) );
  B[2][2] =
      cq3 * ( -3.147678222975001E-2 ) - sq3 * 6.1816327320825E-2 -
      ( clx * mL + 7.735484874E-3 ) *
          ( ( cq35 - cq4 * sq35 ) *
                ( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
                  cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) *
                2.0 +
            ( sq6 * ( cq3 * sq5 + cq45 * sq3 ) - cq6 * sq34 ) *
                ( sq34 * 0.088 + sq45 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) * 2.0 ) -
      ( ( cq6 * ( cq3 * sq5 + cq45 * sq3 ) + sq34 * sq6 ) *
            ( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
              cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) *
            2.0 +
        ( sq6 * ( cq3 * sq5 + cq45 * sq3 ) - cq6 * sq34 ) *
            ( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
              cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) *
            2.0 ) *
          ( clz * mL + 4.5305948634E-2 ) -
      cq3 * ( cq3 * ( -8.626999999999999E-3 ) + cq4 * sq3 * 4.037E-3 + sq34 * 2.29E-4 ) +
      sq3 * ( cq3 * 7.796E-3 + sq3 * 2.5853E-2 ) +
      ( cq35 - cq4 * sq35 ) * ( ( sq6 * ( cq3 * sq5 + cq45 * sq3 ) - cq6 * sq34 ) * ( YZL - 7.41E-4 ) +
                                ( ZZL + 4.815E-3 ) * ( cq35 - cq4 * sq35 ) -
                                ( cq6 * ( cq3 * sq5 + cq45 * sq3 ) + sq34 * sq6 ) * ( XZL - 1.196E-3 ) ) +
      ( mL + 7.35522E-1 ) * pow( sq34 * 0.088 + sq45 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ), 2.0 ) +
      ( cly * mL - 3.127439544E-3 ) *
          ( ( cq35 - cq4 * sq35 ) *
                ( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
                  cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) *
                2.0 -
            ( cq6 * ( cq3 * sq5 + cq45 * sq3 ) + sq34 * sq6 ) *
                ( sq34 * 0.088 + sq45 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) * 2.0 ) +
      ( cq35 - cq4 * sq35 ) *
          ( cq35 * 4.354E-3 + cq3 * sq5 * 1.09E-4 + sq34 * 3.41E-4 + cq45 * sq3 * 1.09E-4 - cq4 * sq35 * 4.354E-3 ) +
      ( mL + 7.35522E-1 ) *
          pow( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
                   cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ),
               2.0 ) +
      ( mL + 7.35522E-1 ) *
          pow( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
                   cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ),
               2.0 ) +
      ( sq6 * ( cq3 * sq5 + cq45 * sq3 ) - cq6 * sq34 ) *
          ( ( sq6 * ( cq3 * sq5 + cq45 * sq3 ) - cq6 * sq34 ) * ( YYL + 1.0027E-2 ) -
            ( cq6 * ( cq3 * sq5 + cq45 * sq3 ) + sq34 * sq6 ) * ( XYL - 4.28E-4 ) +
            ( YZL - 7.41E-4 ) * ( cq35 - cq4 * sq35 ) ) +
      pcq4 * pow( cq3 * 0.0825 + sq3 * 0.384 - 0.0825, 2.0 ) * 2.892501 +
      ( cq3 * sq5 + cq45 * sq3 ) *
          ( cq35 * 1.09E-4 + cq3 * sq5 * 1.964E-3 - sq34 * 1.158E-3 + cq45 * sq3 * 1.964E-3 - cq4 * sq35 * 1.09E-4 ) -
      ( cq6 * ( cq3 * sq5 + cq45 * sq3 ) + sq34 * sq6 ) *
          ( ( cq35 - cq4 * sq35 ) * ( XZL - 1.196E-3 ) +
            ( sq6 * ( cq3 * sq5 + cq45 * sq3 ) - cq6 * sq34 ) * ( XYL - 4.28E-4 ) -
            ( cq6 * ( cq3 * sq5 + cq45 * sq3 ) + sq34 * sq6 ) * ( XXL + 1.2516E-2 ) ) +
      psq4 * pow( cq3 * 0.0825 + sq3 * 0.384 - 0.0825, 2.0 ) * 1.225946 + cq3 * ( cq3 * 1.9552E-2 + sq3 * 7.796E-3 ) +
      cq4 * sq3 * ( cq3 * ( -4.037E-3 ) + cq4 * sq3 * 3.5549E-2 + sq34 * 2.117E-3 ) +
      pcq5 * psq4 * pow( cq3 * 0.0825 + sq3 * 0.384 - 0.0825, 2.0 ) * 1.666555 +
      psq4 * psq5 * pow( cq3 * 0.0825 + sq3 * 0.384 - 0.0825, 2.0 ) * 1.666555 -
      pcq4 * sq3 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 9.4243372804E-2 -
      sq3 * psq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 9.4243372804E-2 -
      cq4 * ( cq3 * sq5 + cq45 * sq3 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 4.705351387E-2 -
      cq4 * ( cq35 - cq4 * sq35 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 2.0048323339E-1 -
      sq34 * ( cq35 * ( -3.41E-4 ) + cq3 * sq5 * 1.158E-3 - sq34 * 5.433E-3 + cq45 * sq3 * 1.158E-3 +
               cq4 * sq35 * 3.41E-4 ) +
      sq34 * ( cq3 * ( -2.29E-4 ) + cq4 * sq3 * 2.117E-3 + sq34 * 2.9474E-2 ) +
      cq34 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 2.9307465076E-2 +
      cq3 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 1.0068694498E-1 -
      cq5 * sq3 * psq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 4.705351387E-2 +
      sq3 * psq4 * sq5 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 2.0048323339E-1 +
      cq5 * sq4 * ( cq35 - cq4 * sq35 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 3.505431787E-2 +
      sq45 * ( cq3 * sq5 + cq45 * sq3 ) * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) * 3.505431787E-2 + 3.525011034375E-2;
  B[2][3] =
      cq3 * 8.640999999999999E-3 - sq3 * 3.449E-3 + cq34 * 2.58333224672E-2 - cq3 * sq4 * 1.481033294592E-3 -
      cq4 * sq3 * 5.5993270925625E-3 + cq5 * sq3 * 1.158E-3 + sq34 * 1.208932934385E-3 + sq35 * 3.41E-4 +
      pcq4 * sq3 * 4.234E-3 - sq34 * sq5 * 1.9409574471375E-3 - cq3 * pcq5 * sq4 * 2.18E-4 -
      pcq4 * cq5 * sq3 * 2.316E-3 - pcq4 * sq35 * 6.82E-4 + cq3 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) +
      cq5 * sq3 * ( XYL - 4.28E-4 ) + cq34 * cq5 * 3.41E-4 - cq34 * sq5 * 1.158E-3 + cq35 * sq4 * 3.849278081088E-2 -
      cq4 * sq34 * 4.995999999999997E-3 + cq3 * sq45 * 9.03427466304E-3 - cq5 * sq34 * 8.269933377337502E-3 -
      cq4 * sq34 * ( ZZL + 4.815E-3 ) + cq3 * sq46 * ( YZL - 7.41E-4 ) - cq6 * sq35 * ( YZL - 7.41E-4 ) -
      cq34 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) + cq4 * pcq5 * sq34 * 2.39E-3 -
      cq5 * sq34 * ( mL * 7.26E-3 + 5.33988972E-3 ) + cq4 * cq6 * sq3 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
      cq4 * sq36 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) + cq3 * sq45 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) -
      cq34 * sq5 * ( XYL - 4.28E-4 ) - cq34 * cq6 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq36 * sq4 * ( XZL - 1.196E-3 ) + cq4 * sq34 * ( XXL + 1.2516E-2 ) - sq35 * sq6 * ( XZL - 1.196E-3 ) -
      cq3 * pcq5 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq5 * cq6 * sq3 * ( cly * mL * 0.088 - 2.75214679872E-4 ) - pcq4 * cq5 * sq3 * ( XYL - 4.28E-4 ) * 2.0 -
      cq5 * pcq6 * sq3 * ( XYL - 4.28E-4 ) * 2.0 + cq5 * sq36 * ( clx * mL * 0.088 + 6.80722668912E-4 ) +
      cq35 * sq45 * 2.39E-3 - sq34 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
      cq35 * sq4 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) -
      pcq4 * cq5 * sq36 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 - cq4 * pcq5 * sq34 * ( YYL + 1.0027E-2 ) +
      cq4 * pcq6 * sq34 * ( YYL + 1.0027E-2 ) - cq5 * cq6 * sq34 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq4 * pcq5 * sq34 * ( ZZL + 4.815E-3 ) + cq5 * sq34 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq34 * cq5 * sq6 * ( XZL - 1.196E-3 ) - cq3 * pcq5 * sq46 * ( YZL - 7.41E-4 ) * 2.0 +
      pcq4 * cq6 * sq35 * ( YZL - 7.41E-4 ) * 2.0 + cq4 * pcq5 * sq34 * ( mL + 7.35522E-1 ) * 7.744E-3 -
      cq5 * cq6 * sq36 * ( XXL + 1.2516E-2 ) - cq35 * sq46 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq34 * cq6 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + cq34 * pcq6 * sq5 * ( XYL - 4.28E-4 ) * 2.0 -
      cq34 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + cq3 * pcq5 * cq6 * sq4 * ( XZL - 1.196E-3 ) * 2.0 -
      cq35 * sq45 * ( YYL + 1.0027E-2 ) + cq5 * cq6 * sq36 * ( YYL + 1.0027E-2 ) + cq45 * sq34 * sq5 * 2.18E-4 -
      cq4 * pcq6 * sq34 * ( XXL + 1.2516E-2 ) + pcq4 * sq35 * sq6 * ( XZL - 1.196E-3 ) * 2.0 -
      cq34 * cq5 * cq6 * ( YZL - 7.41E-4 ) + cq35 * sq45 * ( ZZL + 4.815E-3 ) +
      cq35 * cq6 * sq4 * ( clx * mL * 0.384 + 2.970426191616E-3 ) -
      pcq4 * cq5 * cq6 * sq3 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 +
      pcq4 * cq5 * pcq6 * sq3 * ( XYL - 4.28E-4 ) * 4.0 + cq35 * sq45 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq45 * sq34 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 +
      cq4 * cq6 * sq34 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + cq34 * cq6 * sq56 * ( XXL + 1.2516E-2 ) +
      cq4 * pcq5 * pcq6 * sq34 * ( YYL + 1.0027E-2 ) +
      cq35 * cq6 * sq45 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 - cq34 * cq6 * sq56 * ( YYL + 1.0027E-2 ) -
      cq35 * sq45 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq35 * pcq6 * sq45 * ( XXL + 1.2516E-2 ) +
      pcq4 * cq5 * cq6 * sq36 * ( XXL + 1.2516E-2 ) * 2.0 +
      cq4 * pcq5 * cq6 * sq34 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq35 * pcq6 * sq45 * ( YYL + 1.0027E-2 ) - pcq4 * cq5 * cq6 * sq36 * ( YYL + 1.0027E-2 ) * 2.0 -
      cq4 * pcq5 * sq34 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq4 * pcq5 * pcq6 * sq34 * ( XXL + 1.2516E-2 ) + cq35 * cq6 * sq45 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq45 * cq6 * sq34 * sq5 * ( XZL - 1.196E-3 ) * 2.0 + cq4 * pcq5 * cq6 * sq34 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
      cq45 * sq34 * sq56 * ( YZL - 7.41E-4 ) * 2.0;
  B[2][4] =
      cq3 * 1.0591E-2 - cq4 * 1.208932934385E-3 - sq4 * 5.5993270925625E-3 + cq34 * 1.208932934385E-3 +
      cq45 * 8.269933377337502E-3 + cq3 * ( YYL + 1.0027E-2 ) + cq3 * sq4 * 5.5993270925625E-3 +
      cq4 * sq3 * 1.481033294592E-3 + cq4 * sq5 * 1.9409574471375E-3 + sq34 * 2.58333224672E-2 + cq3 * pcq5 * 2.39E-3 -
      sq34 * sq5 * 1.158E-3 + cq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq3 * pcq5 * ( YYL + 1.0027E-2 ) -
      cq3 * pcq6 * ( YYL + 1.0027E-2 ) + cq4 * pcq5 * sq3 * 2.18E-4 -
      cq4 * sq3 * ( clz * mL * 0.088 + 3.986923479792E-3 ) + cq3 * pcq5 * ( ZZL + 4.815E-3 ) +
      cq6 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) + sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq3 * pcq5 * ( mL + 7.35522E-1 ) * 7.744E-3 - cq34 * cq5 * 8.269933377337502E-3 -
      cq34 * sq5 * 1.9409574471375E-3 - cq45 * sq3 * 3.849278081088E-2 + cq35 * sq5 * 2.18E-4 +
      cq3 * pcq6 * ( XXL + 1.2516E-2 ) + cq4 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) -
      cq4 * sq35 * 9.03427466304E-3 + cq5 * sq34 * 3.41E-4 +
      cq3 * pcq5 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 - cq4 * sq36 * ( YZL - 7.41E-4 ) -
      cq34 * cq5 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq3 * pcq5 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 -
      cq3 * pcq5 * pcq6 * ( XXL + 1.2516E-2 ) + cq45 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
      sq34 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
      cq35 * sq5 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      cq36 * sq4 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) -
      cq3 * sq46 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) - cq4 * sq35 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) -
      cq36 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + cq4 * cq6 * sq3 * ( XZL - 1.196E-3 ) +
      cq3 * pcq5 * pcq6 * ( YYL + 1.0027E-2 ) - sq34 * sq5 * ( XYL - 4.28E-4 ) -
      cq6 * sq34 * ( cly * mL * 0.384 - 1.200936784896E-3 ) +
      cq4 * pcq5 * sq3 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 -
      cq34 * sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) - cq45 * sq35 * 2.39E-3 -
      cq45 * sq3 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) - cq34 * cq5 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) +
      cq34 * cq5 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq35 * cq6 * sq5 * ( XZL - 1.196E-3 ) * 2.0 +
      cq4 * pcq5 * sq36 * ( YZL - 7.41E-4 ) * 2.0 + cq45 * sq36 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
      cq5 * sq34 * sq6 * ( XZL - 1.196E-3 ) + cq3 * pcq5 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 -
      cq4 * pcq5 * cq6 * sq3 * ( XZL - 1.196E-3 ) * 2.0 + cq45 * sq35 * ( YYL + 1.0027E-2 ) -
      cq6 * sq34 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + pcq6 * sq34 * sq5 * ( XYL - 4.28E-4 ) * 2.0 -
      sq34 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) - cq45 * sq35 * ( ZZL + 4.815E-3 ) -
      cq5 * cq6 * sq34 * ( YZL - 7.41E-4 ) + cq35 * sq56 * ( YZL - 7.41E-4 ) * 2.0 -
      cq45 * cq6 * sq3 * ( clx * mL * 0.384 + 2.970426191616E-3 ) - cq45 * sq35 * ( mL + 7.35522E-1 ) * 7.744E-3 +
      cq6 * sq34 * sq56 * ( XXL + 1.2516E-2 ) - cq45 * cq6 * sq35 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 +
      cq45 * sq35 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 + cq45 * pcq6 * sq35 * ( XXL + 1.2516E-2 ) -
      cq6 * sq34 * sq56 * ( YYL + 1.0027E-2 ) - cq45 * pcq6 * sq35 * ( YYL + 1.0027E-2 ) -
      cq45 * cq6 * sq35 * sq6 * ( XYL - 4.28E-4 ) * 2.0;
  B[2][5] = cq35 * 3.41E-4 - cq3 * sq5 * 1.158E-3 + cq5 * sq4 * 1.9409574471375E-3 + sq34 * 5.433E-3 -
            sq45 * 8.269933377337502E-3 + sq34 * sq5 * 3.849278081088E-2 + sq34 * ( mL + 7.35522E-1 ) * 7.744E-3 -
            sq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) - cq3 * sq5 * ( XYL - 4.28E-4 ) + sq34 * ( XXL + 1.2516E-2 ) -
            cq35 * sq4 * 1.9409574471375E-3 - cq45 * sq3 * 1.158E-3 +
            cq5 * sq4 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) + cq3 * sq45 * 8.269933377337502E-3 -
            cq4 * sq35 * 3.41E-4 - cq5 * sq34 * 9.03427466304E-3 + sq34 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
            cq3 * sq45 * ( mL * 7.26E-3 + 5.33988972E-3 ) + pcq6 * sq34 * ( YYL + 1.0027E-2 ) -
            cq6 * sq45 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
            cq5 * sq34 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) - cq45 * sq3 * ( XYL - 4.28E-4 ) +
            sq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) - cq35 * sq6 * ( XZL - 1.196E-3 ) -
            cq36 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + cq3 * pcq6 * sq5 * ( XYL - 4.28E-4 ) * 2.0 -
            cq35 * sq4 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
            cq6 * sq34 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
            cq3 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
            sq34 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - pcq6 * sq34 * ( XXL + 1.2516E-2 ) -
            cq35 * cq6 * ( YZL - 7.41E-4 ) + cq6 * sq34 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
            cq36 * sq45 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
            cq3 * sq45 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) + cq6 * sq34 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
            cq36 * sq56 * ( XXL + 1.2516E-2 ) + cq4 * sq35 * sq6 * ( XZL - 1.196E-3 ) -
            sq34 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
            cq45 * cq6 * sq3 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + cq45 * pcq6 * sq3 * ( XYL - 4.28E-4 ) * 2.0 -
            cq45 * sq36 * ( clx * mL * 0.088 + 6.80722668912E-4 ) - cq36 * sq56 * ( YYL + 1.0027E-2 ) +
            cq4 * cq6 * sq35 * ( YZL - 7.41E-4 ) + cq45 * cq6 * sq36 * ( XXL + 1.2516E-2 ) -
            cq45 * cq6 * sq36 * ( YYL + 1.0027E-2 );
  B[2][6] = -( cly * mL - 3.127439544E-3 ) *
                ( sq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) -
                  cq5 * cq6 * sq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) ) -
            ( sq6 * ( cq3 * sq5 + cq45 * sq3 ) - cq6 * sq34 ) * ( YZL - 7.41E-4 ) -
            ( ZZL + 4.815E-3 ) * ( cq35 - cq4 * sq35 ) +
            ( cq6 * ( cq3 * sq5 + cq45 * sq3 ) + sq34 * sq6 ) * ( XZL - 1.196E-3 ) +
            ( clx * mL + 7.735484874E-3 ) *
                ( cq6 * ( cq35 * ( -0.088 ) + cq4 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) + cq4 * sq35 * 0.088 ) +
                  cq5 * sq46 * ( cq3 * 0.0825 + sq3 * 0.384 - 0.0825 ) );
  B[3][3] =
      cq4 * 2.41786586877E-3 - cq5 * 1.806854932608E-2 + sq4 * 1.1198654185125E-2 + sq5 * 7.698556162176001E-2 -
      cq45 * 1.6539866754675E-2 + pow( cq5 * 0.0825 - cq4 * sq5 * 0.384, 2.0 ) * 1.666555 +
      pow( sq5 * 0.0825 + cq45 * 0.384, 2.0 ) * 1.666555 - cq4 * sq5 * 3.881914894275E-3 +
      ( cq4 * cq6 + cq5 * sq46 ) *
          ( -sq45 * ( YZL - 7.41E-4 ) + cq4 * sq6 * ( XYL - 4.28E-4 ) + cq4 * cq6 * ( YYL + 1.0027E-2 ) -
            cq5 * cq6 * sq4 * ( XYL - 4.28E-4 ) + cq5 * sq46 * ( YYL + 1.0027E-2 ) ) +
      cq4 * ( cq4 * 5.433E-3 + cq5 * sq4 * 1.158E-3 + sq45 * 3.41E-4 ) + pcq4 * 1.80773093376E-1 +
      psq4 * 4.26516627456E-1 - sq4 * ( cq4 * 2.117E-3 - sq4 * 3.5549E-2 ) +
      ( ( cq4 * cq6 + cq5 * sq46 ) * ( cq4 * 0.088 - cq5 * 0.0825 + cq4 * sq5 * 0.384 ) * 2.0 -
        sq45 * ( sq6 * ( sq5 * 0.0825 + cq45 * 0.384 ) - cq6 * sq4 * ( sq5 * 0.088 + 0.384 ) ) * 2.0 ) *
          ( clx * mL + 7.735484874E-3 ) -
      ( clz * mL + 4.5305948634E-2 ) *
          ( cq5 * 0.768 + cq4 * sq5 * 0.165 + cq5 * sq5 * 0.176 - pcq4 * cq5 * sq5 * 0.176 ) +
      cq4 * ( cq4 * 2.9474E-2 - sq4 * 2.117E-3 ) +
      ( mL + 7.35522E-1 ) * pow( cq4 * 0.088 - cq5 * 0.0825 + cq4 * sq5 * 0.384, 2.0 ) -
      ( ( cq4 * sq6 - cq5 * cq6 * sq4 ) * ( cq4 * 0.088 - cq5 * 0.0825 + cq4 * sq5 * 0.384 ) * 2.0 +
        sq45 * ( cq6 * ( sq5 * 0.0825 + cq45 * 0.384 ) + sq46 * ( sq5 * 0.088 + 0.384 ) ) * 2.0 ) *
          ( cly * mL - 3.127439544E-3 ) +
      ( cq4 * sq6 - cq5 * cq6 * sq4 ) *
          ( cq4 * cq6 * ( XYL - 4.28E-4 ) + cq4 * sq6 * ( XXL + 1.2516E-2 ) - sq45 * ( XZL - 1.196E-3 ) +
            cq5 * sq46 * ( XYL - 4.28E-4 ) - cq5 * cq6 * sq4 * ( XXL + 1.2516E-2 ) ) +
      pow( cq6 * ( sq5 * 0.0825 + cq45 * 0.384 ) + sq46 * ( sq5 * 0.088 + 0.384 ), 2.0 ) * ( mL + 7.35522E-1 ) +
      pow( sq6 * ( sq5 * 0.0825 + cq45 * 0.384 ) - cq6 * sq4 * ( sq5 * 0.088 + 0.384 ), 2.0 ) * ( mL + 7.35522E-1 ) -
      sq45 * ( cq4 * cq6 * ( YZL - 7.41E-4 ) - sq45 * ( ZZL + 4.815E-3 ) + cq4 * sq6 * ( XZL - 1.196E-3 ) +
               cq5 * sq46 * ( YZL - 7.41E-4 ) - cq5 * cq6 * sq4 * ( XZL - 1.196E-3 ) ) +
      cq5 * sq4 * ( cq4 * 1.158E-3 + cq5 * sq4 * 1.964E-3 - sq45 * 1.09E-4 ) +
      sq45 * ( cq4 * 3.41E-4 - cq5 * sq4 * 1.09E-4 + sq45 * 4.354E-3 ) + 4.776398057640078E-4;
  B[3][4] = cq4 * 2.58333224672E-2 - sq4 * 1.481033294592E-3 + cq45 * 3.41E-4 - cq4 * sq5 * 1.158E-3 +
            cq5 * sq4 * 3.849278081088E-2 + sq45 * 9.03427466304E-3 - pcq5 * sq4 * 2.18E-4 +
            sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) + cq5 * sq4 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
            sq46 * ( YZL - 7.41E-4 ) - cq4 * sq6 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
            sq45 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) - cq4 * sq5 * ( XYL - 4.28E-4 ) -
            cq4 * cq6 * ( cly * mL * 0.384 - 1.200936784896E-3 ) - cq6 * sq4 * ( XZL - 1.196E-3 ) -
            pcq5 * sq4 * ( clz * mL * 0.088 + 3.986923479792E-3 ) * 2.0 + cq5 * sq45 * 2.39E-3 +
            cq5 * sq45 * ( ZZL + 4.815E-3 ) + cq5 * cq6 * sq4 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
            cq5 * sq45 * ( mL + 7.35522E-1 ) * 7.744E-3 - cq45 * sq6 * ( XZL - 1.196E-3 ) -
            pcq5 * sq46 * ( YZL - 7.41E-4 ) * 2.0 - cq5 * sq46 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
            cq4 * cq6 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) + cq4 * pcq6 * sq5 * ( XYL - 4.28E-4 ) * 2.0 -
            cq4 * sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + pcq5 * cq6 * sq4 * ( XZL - 1.196E-3 ) * 2.0 -
            cq5 * sq45 * ( YYL + 1.0027E-2 ) - cq45 * cq6 * ( YZL - 7.41E-4 ) +
            cq5 * pcq6 * sq45 * ( YYL + 1.0027E-2 ) + cq4 * cq6 * sq56 * ( XXL + 1.2516E-2 ) +
            cq5 * cq6 * sq45 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 - cq4 * cq6 * sq56 * ( YYL + 1.0027E-2 ) -
            cq5 * sq45 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq5 * pcq6 * sq45 * ( XXL + 1.2516E-2 ) +
            cq5 * cq6 * sq45 * sq6 * ( XYL - 4.28E-4 ) * 2.0;
  B[3][5] = cq4 * 5.433E-3 - cq5 * 8.269933377337502E-3 - sq5 * 1.9409574471375E-3 - cq45 * 9.03427466304E-3 +
            cq4 * sq5 * 3.849278081088E-2 + cq5 * sq4 * 1.158E-3 - sq5 * ( clz * mL * 0.0825 + 3.737740762305E-3 ) +
            sq45 * 3.41E-4 + cq4 * ( mL + 7.35522E-1 ) * 7.744E-3 - cq5 * ( mL * 7.26E-3 + 5.33988972E-3 ) +
            cq4 * ( XXL + 1.2516E-2 ) + cq4 * sq5 * ( mL * 3.3792E-2 + 2.4854759424E-2 ) +
            cq4 * pcq6 * ( YYL + 1.0027E-2 ) - cq5 * cq6 * ( clx * mL * 0.0825 + 6.38177502105E-4 ) -
            cq45 * ( clz * mL * 0.384 + 1.7397484275456E-2 ) + cq5 * sq6 * ( cly * mL * 0.0825 - 2.5801376238E-4 ) +
            cq5 * sq4 * ( XYL - 4.28E-4 ) + cq4 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
            cq4 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 - cq4 * pcq6 * ( XXL + 1.2516E-2 ) -
            cq6 * sq45 * ( YZL - 7.41E-4 ) + cq4 * cq6 * sq5 * ( clx * mL * 0.384 + 2.970426191616E-3 ) +
            cq4 * cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 - cq4 * sq56 * ( cly * mL * 0.384 - 1.200936784896E-3 ) -
            sq45 * sq6 * ( XZL - 1.196E-3 ) + cq5 * cq6 * sq4 * ( cly * mL * 0.088 - 2.75214679872E-4 ) -
            cq5 * pcq6 * sq4 * ( XYL - 4.28E-4 ) * 2.0 + cq5 * sq46 * ( clx * mL * 0.088 + 6.80722668912E-4 ) -
            cq5 * cq6 * sq46 * ( XXL + 1.2516E-2 ) + cq5 * cq6 * sq46 * ( YYL + 1.0027E-2 );
  B[3][6] =
      ( sq6 * ( sq5 * 0.0825 + cq45 * 0.384 ) - cq6 * sq4 * ( sq5 * 0.088 + 0.384 ) ) * ( clx * mL + 7.735484874E-3 ) +
      ( cq4 * sq6 - cq5 * cq6 * sq4 ) * ( XZL - 1.196E-3 ) +
      ( cq6 * ( sq5 * 0.0825 + cq45 * 0.384 ) + sq46 * ( sq5 * 0.088 + 0.384 ) ) * ( cly * mL - 3.127439544E-3 ) +
      ( YZL - 7.41E-4 ) * ( cq4 * cq6 + cq5 * sq46 ) - sq45 * ( ZZL + 4.815E-3 );
  B[4][4] = sin( q[5] * 2.0 ) * 1.09E-4 + pcq5 * 4.354E-3 + psq5 * 1.964E-3 + pcq5 * ( ZZL + 4.815E-3 ) +
            pcq5 * ( mL + 7.35522E-1 ) * 7.744E-3 + sin( q[5] * 2.0 ) * ( clz * mL * 0.088 + 3.986923479792E-3 ) +
            pcq5 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
            pcq5 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 + pcq6 * psq5 * ( XXL + 1.2516E-2 ) +
            psq5 * psq6 * ( YYL + 1.0027E-2 ) + cq5 * sq56 * ( YZL - 7.41E-4 ) * 2.0 -
            cq5 * cq6 * sq5 * ( XZL - 1.196E-3 ) * 2.0 - cq6 * psq5 * sq6 * ( XYL - 4.28E-4 ) * 2.0 +
            8.626999999999999E-3;
  B[4][5] = cq5 * 3.41E-4 - sq5 * 1.158E-3 - sq5 * ( XYL - 4.28E-4 ) - cq5 * cq6 * ( YZL - 7.41E-4 ) -
            cq5 * sq6 * ( XZL - 1.196E-3 ) - cq6 * sq5 * ( cly * mL * 0.088 - 2.75214679872E-4 ) +
            pcq6 * sq5 * ( XYL - 4.28E-4 ) * 2.0 - sq56 * ( clx * mL * 0.088 + 6.80722668912E-4 ) +
            cq6 * sq56 * ( XXL + 1.2516E-2 ) - cq6 * sq56 * ( YYL + 1.0027E-2 );
  B[4][6] = -cq5 * ( ZZL + 4.815E-3 ) - sq56 * ( YZL - 7.41E-4 ) + cq6 * sq5 * ( XZL - 1.196E-3 ) -
            cq5 * cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) + cq5 * sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 );
  B[5][5] = mL * 7.744E-3 + cq6 * ( clx * mL * 0.088 + 6.80722668912E-4 ) * 2.0 -
            sq6 * ( cly * mL * 0.088 - 2.75214679872E-4 ) * 2.0 + psq6 * ( XXL + 1.2516E-2 ) +
            pcq6 * ( YYL + 1.0027E-2 ) + cq6 * sq6 * ( XYL - 4.28E-4 ) * 2.0 + 1.1128882368E-2;
  B[5][6] = cq6 * ( YZL - 7.41E-4 ) + sq6 * ( XZL - 1.196E-3 );
  B[6][6] = ZZL + 4.815E-3;
  B[1][0] = B[0][1];
  B[2][0] = B[0][2];
  B[2][1] = B[1][2];
  B[3][0] = B[0][3];
  B[3][1] = B[1][3];
  B[3][2] = B[2][3];
  B[4][0] = B[0][4];
  B[4][1] = B[1][4];
  B[4][2] = B[2][4];
  B[4][3] = B[3][4];
  B[5][0] = B[0][5];
  B[5][1] = B[1][5];
  B[5][2] = B[2][5];
  B[5][3] = B[3][5];
  B[5][4] = B[4][5];
  B[6][0] = B[0][6];
  B[6][1] = B[1][6];
  B[6][2] = B[2][6];
  B[6][3] = B[3][6];
  B[6][4] = B[4][6];
  B[6][5] = B[5][6];

  return B;
}
} // namespace franka_model
