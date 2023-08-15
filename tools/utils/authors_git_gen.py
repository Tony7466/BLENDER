#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Blender Foundation
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""
Example use:

   authors_git_gen.py --source=/src/blender --range=SHA1..HEAD
"""

import argparse
import multiprocessing
import re
import unicodedata

from typing import (
    Dict,
    Tuple,
    Iterable,
    List,
)

from git_log import (
    GitCommitIter,
    GitCommit,
)

# Exclude authors with this many lines modified.
AUTHOR_LINES_SKIP = 4

# Stop counting after this line limit.
AUTHOR_LINES_LIMIT = 100

# -----------------------------------------------------------------------------
# Lookup Table to clean up the authors
#
# This is a combination of unifying git logs as well as
# name change requested by the authors.

author_table_source = {
    "Aaron Carlisle <carlisle.aaron00@gmail.com>": (
        "Aaron <Aaron>",
        "Aaron <carlisle.b3d@gmail.com>",
        "Aaron Carlisle <Blendify>",
        "Aaron Carlisle <blendify@noreply.localhost>",
        "Aaron Carlisle <carlisle.b3d@gmail.com>",
        "Your Name <Aaron Carlisle>",
    ),
    "Alaska <alaskayou01@gmail.com>": (
        "Alaska <Alaska>",
        "Alaska <alaska@noreply.localhost>",
    ),
    "Aleksandr Zinovev <roaoao@gmail.com>": (
        "raa <roaoao@gmail.com>",
    ),
    "Alexander Gavrilov <angavrilov@gmail.com>": (
        "Alexander Gavrilov <alexander.gavrilov@jetbrains.com>",
    ),
    "Alexander Romanov <a.romanov@blend4web.com>": (
        "Romanov Alexander <a.romanov@blend4web.com>",
    ),
    "Alexandr Kuznetsov <ak3636@nyu.edu>": (
        "Alexandr Kuznetsov <kuzsasha@gmail.com>",
    ),
    "Amélie Fondevilla <amelie.fondevilla@les-fees-speciales.coop>": (
        "Amelie <amelie.fondevilla@les-fees-speciales.coop>",
        "Amelie Fondevilla <afonde>",
        "Amelie Fondevilla <amelie.fondevilla@les-fees-speciales.coop>",
        "Amelie Fondevilla <amelie@les-fees-speciales.coop>",
        "Amelie Fondevilla <amelief@noreply.localhost>",
        "Amélie Fondevilla <afonde>",
    ),
    "Andrii Symkin <pembem22>": (
        "Andrii <pembem22>",
    ),
    "Ankit Meel <ankitjmeel@gmail.com>": (
        "Ankit <ankitm>",
        "Ankit Meel <ankitm>",
    ),
    "Anthony Edlin <akrashe@gmail.com>": (
        "Anthony Edlin <krash>",
    ),
    "Antonio Vazquez <blendergit@gmail.com>": (
        "Antonio  Vazquez <blendergit@gmail.com>",
        "Antonio Vazquez <antoniov>",
        "Antonio Vazquez <antoniov@noreply.localhost>",
        "Antonioya <blendergit@gmail.com>",
    ),
    "Antony Riakiotakis <kalast@gmail.com>": (
        "Antony Ryakiotakis <kalast@gmail.com>",
    ),
    "Aras Pranckevicius <aras@nesnausk.org>": (
        "Aras Pranckevicius <aras_p>",
    ),
    "Bastien Montagne <bastien@blender.org>": (
        "Bastien Montagne (@mont29) <>",
        "Bastien Montagne <b.mont29@gmail.com>",
        "Bastien Montagne <mont29>",
        "Bastien Montagne <montagne29@wanadoo.fr>",
        "bastien <bastien@blender.org>",
        "mont29 <montagne29@wanadoo.fr>",
    ),
    "Bogdan Nagirniak <bodyan@gmail.com>": (
        "Bogdan Nagirniak <bnagirniak>",
    ),
    "Brecht Van Lommel <brecht@blender.org>": (
        "Brecht Van Lommel <brecht>",
        "Brecht Van Lommel <brecht@noreply.localhost>",
        "Brecht Van Lommel <brecht@solidangle.com>",
        "Brecht Van Lommel <brechtvanlommel@gmail.com>",
        "Brecht Van Lommel <brechtvanlommel@pandora.be>",
        "Brecht Van Lömmel <brechtvanlommel@gmail.com>",
        "Brecht van Lommel <brechtvanlommel@gmail.com>",
        "recht Van Lommel <brecht@blender.org>",
    ),
    "Brendon Murphy <meta.androcto1@gmail.com>": (
        "meta-androcto <meta.androcto1@gmail.com>",
    ),
    "Brian Savery <brian.savery@gmail.com>": (
        "Brian Savery <bsavery>",
    ),
    "Campbell Barton <campbell@blender.org>": (
        "Campbell Barton <campbellbarton>",
        "Campbell Barton <ideasman42@gmail.com>",
    ),
    "Casey Bianco-Davis <caseycasey739@gmail.com>": (
        "casey bianco-davis <caseycasey739@gmail.com>",
    ),
    "Charles Wardlaw <cwardlaw@nvidia.com>": (
        "Charles Wardlaw (@CharlesWardlaw) <>",
        "Charles Wardlaw <charleswardlaw@noreply.localhost>",
        "Charles Wardlaw <kattkieru@users.noreply.github.com>",
    ),
    "Charlie Jolly <mistajolly@gmail.com>": (
        "Charlie Jolly <charlie>",
        "charlie <mistajolly@gmail.com>",
    ),
    "Christian Brinkmann <hallo@zblur.de>": (
        "christian brinkmann <>",
    ),
    "Christian Rauch <Rauch.Christian@gmx.de>": (
        "Christian Rauch <christian.rauch>",
    ),
    "Christoph Lendenfeld <chris.lenden@gmail.com>": (
        "Christoph Lendenfeld <ChrisLend>",
        "Christoph Lendenfeld <chris.lend@gmx.at>",
        "Christoph Lendenfeld <chrislend@noreply.localhost>",
    ),
    "Clément Foucault <foucault.clem@gmail.com>": (
        "Clment Foucault <fclem>",
        "Clment Foucault <foucault.clem@gmail.com>",
        "ClÃ©ment Foucault <foucault.clem@gmail.com>",
        "Clément <clement@Clements-MacBook-Pro.local>",
        "Clément Foucault <fclem>",
        "Clément Foucault <fclem@noreply.localhost>",
        "fclem <foucault.clem@gmail.com>",
    ),
    "Colin Basnett <cmbasnett@gmail.com>": (
        "Colin Basnett <cmbasnett>",
        "Colin Basnett <cmbasnett@noreply.localhost>",
    ),
    "Colin Marmond <kdblender@gmail.com>": (
        "Colin <Kdaf>",
        "Colin Marmond <Kdaf>",
        "Colin Marmond <kdaf@noreply.localhost>",
        "Colin Marmont <Kdaf>",
    ),
    "Dalai Felinto <dalai@blender.org>": (
        "Dalai Felinto <dfelinto>",
        "Dalai Felinto <dfelinto@gmail.com>",
        "Dalai Felinto <dfelinto@noreply.localhost>",
    ),
    "Damien Picard <dam.pic@free.fr>": (
        "Damien Picard <pioverfour>",
        "Damien Picard <pioverfour@noreply.localhost>",
    ),
    "Daniel Salazar <zanqdo@gmail.com>": (
        "Daniel Salazar <zanqdo>",
        "Daniel Salazar <zanqdo@noreply.localhost>",
        "Daniel Santana <dgsantana>",
        "ZanQdo <zanqdo@gmail.com>",
        "zanqdo <zanqdo@gmail.com>",
    ),
    "David Friedli <hlorus>": (
        "David <hlorus>",
    ),
    "Diego Borghetti <bdiego@gmail.com>": (
        "Diego Hernan Borghetti <bdiego@gmail.com>",
    ),
    "Dilith Jayakody <dilithjay@gmail.com>": (
        "dilithjay <dilithjay@gmail.com>",
    ),
    "Dmitry Dygalo <noreply@developer.blender.org>": (
        "Dmitry Dygalo <>",
    ),
    "EitanSomething <eitant13@gmail.com>": (
        "Eitan <EitanSomething>",
        "EitanSomething <EitanSomething>",
    ),
    "Ejner Fergo <ejnersan@gmail.com>": (
        "Ejner Fergo <ejnersan>",
    ),
    "Erik Abrahamsson <ecke101@gmail.com>": (
        "Eric Abrahamsson <ecke101@gmail.com>",
        "Erick Abrahammson <ecke101@gmail.com>",
        "Erik <ecke101@gmail.com>",
        "Erik Abrahamsson <erik85>",
        "Erik Abrahamsson <erik85@noreply.localhost>",
    ),
    "Ethan Hall <Ethan1080>": (
        "Ethan-Hall <Ethan1080>",
    ),
    "Fabian Schempp <fabianschempp@googlemail.com>": (
        "Fabian Schempp <fabian_schempp>",
    ),
    "Falk David <falk@blender.org>": (
        "Falk David <falkdavid@gmx.de>",
        "Falk David <filedescriptor>",
        "Falk David <filedescriptor@noreply.localhost>",
        "filedescriptor <falkdavid@gmx.de>",
    ),
    "Francesco Siddi <francesco@blender.org>": (
        "Francesco Siddi <francesco.siddi@gmail.com>",
        "Francesco Siddi <fsiddi>",
    ),
    "Fynn Grotehans <fynngr@noreply.localhost>": (
        "Fynn Grotehans <68659993+Fynn-G@users.noreply.github.com>",
        "Fynn Grotehans <TheFynn>",
    ),
    "Félix <Miadim>": (
        "Flix <Miadim>",
    ),
    "Gaia Clary <gaia.clary@machinimatrix.org>": (
        "Gaia Clary <gaiaclary>",
        "gaiaclary <gaia.clary@machinimatrix.org>",
    ),
    "Germano Cavalcante <germano.costa@ig.com.br>": (
        "Germano <germano.costa@ig.com.br>",
        "Germano Cavalcante <grmncv@gmail.com>",
        "Germano Cavalcante <mano-wii>",
        "Germano Cavalcante <mano-wii@noreply.localhost>",
        "Germano Cavalcantemano-wii <germano.costa@ig.com.br>",
        "mano-wii <germano.costa@ig.com.br>",
        "mano-wii <grmncv@gmail.com>",
    ),
    "Guillermo S. Romero <gsr.b3d@infernal-iceberg.com>": (
        "gsr b3d <gsr.b3d@infernal-iceberg.com>",
    ),
    "Guillermo Venegas <guillermovcra@gmail.com>": (
        "Guillermo <guillermovcra@gmail.com>",
        "guishe <guillermovcra@gmail.com>",
    ),
    "Habib Gahbiche <habibgahbiche@gmail.com>": (
        "Habib Gahbiche <zazizizou>",
    ),
    "Hans Goudey <h.goudey@me.com>": (
        "Hans Goudey <HooglyBoogly>",
        "Hans Goudey <hooglyboogly@noreply.localhost>",
    ),
    "Harley Acheson <harley.acheson@gmail.com>": (
        "Harley Acheson <harley>",
        "Harley Acheson <harley@noreply.localhost>",
    ),
    "Henrik Dick <hen-di@web.de>": (
        "Henrik Dick (weasel) <>",
        "Henrik Dick <weasel>",
    ),
    "Himanshi Kalra <himanshikalra98@gmail.com>": (
        "Himanshi Kalra <calra>",
    ),
    "Howard Trickey <howard.trickey@gmail.com>": (
        "howardt <howard.trickey@gmail.com>",
    ),
    "Hristo Gueorguiev <prem.nirved@gmail.com>": (
        "Hristo Gueorguiev <>",
    ),
    "IRIE Shinsuke <irieshinsuke@yahoo.co.jp>": (
        "Irie Shinsuke <irieshinsuke@yahoo.co.jp>",
    ),
    "Iliya Katueshenock <modormoder@gmail.com>": (
        "Iliay Katueshenock <Moder>",
        "Iliya Katueshenock <Moder>",
        "Iliya Katueshenock <mod_moder@noreply.localhost>",
        "MOD <Moder>",
        "illua1 <modormoder@gmail.com>",
    ),
    "Inês Almeida <britalmeida@gmail.com>": (
        "Ines Almeida <britalmeida@gmail.com>",
        "brita <britalmeida@gmail.com>",
    ),
    "Ish Bosamiya <ish_bosamiya>": (
        "Ish Bosamiya <ishbosamiya>",
        "Ish Bosamiya <ishbosamiya@gmail.com>",
    ),
    "Iyad Ahmed <iyadahmed430@gmail.com>": (
        "Iyad Ahmed <iyadahmed2001>",
    ),
    "Jacques Lucke <jacques@blender.org>": (
        "Jacques Lucke <jacqueslucke@noreply.localhost>",
        "Jacques Lucke <mail@jlucke.com>",
    ),
    "Jason Fielder <jason-fielder@noreply.localhost>": (
        "Jason Fielder <jason_apple>",
    ),
    "Jens Ole Wund <bjornmose@gmx.net>": (
        "bjornmose <bjornmose@gmx.net>",
    ),
    "Jens Verwiebe <info@jensverwiebe.de>": (
        "Jens <info@jensverwiebe.de>",
        "Jens Verwiebe <jensverwiebe@jens-macpro.fritz.box>",
        "jensverwiebe <info@jensverwiebe.de>",
    ),
    "Jeroen Bakker <jeroen@blender.org>": (
        "Jeroen Bakker <88891617+jeroen-blender@users.noreply.github.com>",
        "Jeroen Bakker <j.bakker@atmind.nl>",
        "Jeroen Bakker <jbakker>",
    ),
    "Jesse Yurkovich <jesse.y@gmail.com>": (
        "Jesse Y <deadpin>",
        "Jesse Yurkovich <deadpin>",
    ),
    "Johannes J <johannesj@noreply.localhost>": (
        "Johannes J <johannesj>",
    ),
    "Johnny Matthews <johnny.matthews@gmail.com>": (
        "Johnny Matthews (guitargeek) <johnny.matthews@gmail.com>",
        "Johnny Matthews <guitargeek>",
        "guitargeek <johnny.matthews@gmail.com>",
    ),
    "Jorijn de Graaf <bonj@noreply.localhost>": (
        "Jorijn de Graaf <bonj>",
    ),
    "Joseph Eagar <joeedh@gmail.com>": (
        "Joe Eagar <joeedh@gmail.com>",
        "Joseph Eagar <joeedh>",
        "Joseph Eagar <josepheagar@noreply.localhost>",
    ),
    "Josh Maros <joshm-2@noreply.localhost>": (
        "Josh Maros <60271685+joshua-maros@users.noreply.github.com>",
        "joshua-maros <60271685+joshua-maros@users.noreply.github.com>",
    ),
    "Julian Eisel <julian@blender.org>": (
        "Julian Eisel <Severin>",
        "Julian Eisel <eiseljulian@gmail.com>",
        "Julian Eisel <julian@linux-chl2.site>",
        "Julian Eisel <julian_eisel@web.de>",
        "Julian Eisel <julianeisel@Julians-MacBook-Pro.local>",
        "Julian Eisel <julianeisel@noreply.localhost>",
        "Severin <eiseljulian@gmail.com>",
        "Severin <julian_eisel@web.de>",
        "julianeisel <julian_eisel@web.de>",
    ),
    "Jun Mizutani <jmztn@noreply.localhost>": (
        "Jun Mizutani <jmztn>",
    ),
    "Jörg Müller <nexyon@gmail.com>": (
        "Joerg Mueller <nexyon@gmail.com>",
    ),
    "Jürgen Herrmann <shadowrom@me.com>": (
        "Juergen Herrmann <shadowrom@me.com>",
    ),
    "Kaspian Jakobsson <kaspian.jakobsson@gmail.com>": (
        "kaspian.jakobssongmail.com <kaspian.jakobsson@gmail.com>",
    ),
    "Kevin C. Burke <kevincburke@noreply.localhost>": (
        "Kevin C. Burke <blastframe>",
    ),
    "Kévin Dietrich <kevin.dietrich@mailoo.org>": (
        "Kevin Dietrich <kevin.dietrich@mailoo.org>",
        "Kévin Dietrich <kevindietrich>",
    ),
    "Leon Schittek <leon.schittek@gmx.net>": (
        "Leon Leno <lone_noel>",
        "Leon Schittek <lone_noel>",
        "Leon Schittek <lone_noel@noreply.localhost>",
    ),
    "Lictex Steaven <lictex_>": (
        "lictex_ <lictex_>",
    ),
    "Luca Rood <dev@lucarood.com>": (
        "Luca Rood <LucaRood>",
    ),
    "Lukas Stockner <lukas.stockner@freenet.de>": (
        "Lukas Stockner <lukasstockner97>",
    ),
    "Lukas Tönne <lukas@blender.org>": (
        "Lukas Toenne <lukas.toenne@googlemail.com>",
        "Lukas Tönne <lukas.toenne@gmail.com>",
        "Lukas Tönne <lukastonne@noreply.localhost>",
    ),
    "Mai Lavelle <mai.lavelle@gmail.com>": (
        "Mai Lavelle <lavelle@gmail.com>",
    ),
    "Mal Duffin <malachyduffin@gmail.com>": (
        "Mal Duffin <mal_cando>",
    ),
    "Manuel Castilla <manzanillawork@gmail.com>": (
        "Manuel Castilla <manzanilla>",
    ),
    "Marc Chehab <marcchehab@protonmail.ch>": (
        "Marc Chéhab <marcchehab@noreply.localhost>",
        "Marc Chéhab <marcluzmedia>",
    ),
    "Martijn Berger <mberger@denc.com>": (
        "Martijn Berger <martijn.berger@gmail.com>",
        "Martijn Berger <mberger@denc.nl>",
        "Martijn Berger <mberger@martijns-mbp.lan>",
    ),
    "Martijn Versteegh <martijn@aaltjegron.nl>": (
        "Martijn Versteegh <Baardaap>",
        "Martijn Versteegh <baardaap@noreply.localhost>",
        "Martijn Versteegh <blender@aaltjegron.nl>",
    ),
    "Matias Mendiola <matias.mendiola@gmail.com>": (
        "Matias Mendiola <mendio>",
    ),
    "Matt Heimlich <matt.heimlich@gmail.com>": (
        "Matt Heimlich <m9105826>",
    ),
    "Matteo F. Vescovi <mfvescovi@gmail.com>": (
        "Matteo F. Vescovi <mfv>",
    ),
    "Max Schlecht <bobbe@noreply.localhost>": (
        "Max Schlecht <Bobbe>",
    ),
    "Maxime Casas <maxime_casas@orange.fr>": (
        "Maxime Casas <troopy28>",
    ),
    "Michael Jones <michael_p_jones@apple.com>": (
        "Michael Jones (Apple) <michael-jones@noreply.localhost>",
        "Michael Jones <michael_jones>",
    ),
    "Michael Kowalski <makowalski@nvidia.com>": (
        "Michael Kowalski <makowalski>",
        "Michael Kowalski <makowalski@noreply.localhost>",
    ),
    "Miguel Pozo <pragma37@gmail.com>": (
        "Miguel Pozo <pragma37>",
        "Miguel Pozo <pragma37@noreply.localhost>",
    ),
    "Mikhail Matrosov <ktdfly>": (
        "Mikhail <ktdfly>",
        "Mikhail Matrosov <kdtfly>",
    ),
    "Monique Dewanchand <m.dewanchand@atmind.nl>": (
        "Monique Dewanchand <mdewanchand>",
    ),
    "Nate Rupsis <nrupsis@gmail.com>": (
        "Nate Rupsis <C-Nathaniel.Rupsis@charter.com>",
        "Nate Rupsis <C-nathaniel.rupsis@charter.com>",
        "Nate Rupsis <nrupsis>",
        "Nate Rupsis <nrupsis@noreply.localhost>",
    ),
    "Nathan Craddock <nzcraddock@gmail.com>": (
        "Nathan Craddock <Zachman>",
    ),
    "Nathan Letwory <nathan@blender.org>": (
        "Nathan Letwory <jesterking>",
        "Nathan Letwory <nathan@letworyinteractive.com>",
        "Nathan Letwory <nathan@mcneel.com>",
    ),
    "Nathan Vegdahl <cessen@cessen.com>": (
        "Nathan Vegdahl <cessen>",
    ),
    "Nicholas Bishop <nicholasbishop@gmail.com>": (
        "Nicholas Bishop <nicholas.bishop@floored.com>",
    ),
    "Nicholas Rishel <rishel.nick@gmail.com>": (
        "Nicholas Rishel <nicholas_rishel>",
    ),
    "Nick Milios <semaphore>": (
        "milios <n_mhlios@hotmail.com>",
    ),
    "Nikita Sirgienko <nikita.sirgienko@intel.com>": (
        "Nikita Sirgienko <sirgienko>",
    ),
    "Omar Emara <mail@OmarEmara.dev>": (
        "Omar Emara <OmarSquircleArt>",
        "OmarSquircleArt <mail@OmarEmara.dev>",
        "OmarSquircleArt <omar.squircleart@gmail.com>",
    ),
    "Pablo Dobarro <pablodp606@gmail.com>": (
        "Pablo Dobarro <pablodp606>",
    ),
    "Pablo Vazquez <pablo@blender.org>": (
        "Pablo Vazquez <contact@pablovazquez.art>",
        "Pablo Vazquez <pablovazquez>",
        "Pablo Vazquez <venomgfx@gmail.com>",
    ),
    "Patrick Busch <xylvier@noreply.localhost>": (
        "Patrick Busch <Xylvier>",
    ),
    "Patrick Mours <pmours@nvidia.com>": (
        "Patrick Mours <pmoursnv@noreply.localhost>",
    ),
    "Paul Golter <paulgolter>": (
        "Paul Golter <pullpullson>",
    ),
    "Philipp Oeser <philipp@blender.org>": (
        "Philipp Oeser <info@graphics-engineer.com>",
        "Philipp Oeser <lichtwerk>",
        "Philipp Oeser <lichtwerk@noreply.localhost>",
        "Philipp Oeser <noreply@developer.blender.org>",
        "Philipp Oeser <poeser@posteo.de>",
    ),
    "Pratik Borhade <pratikborhade302@gmail.com>": (
        "Pratik Borhade <PratikPB2123>",
    ),
    "Ray Molenkamp <github@lazydodo.com>": (
        "Lazydodo <github@lazydodo.com>",
        "Ray Molenkamp <LazyDodo>",
        "Ray molenkamp <LazyDodo>",
        "Ray molenkamp <lazydodo@noreply.localhost>",
        "lazydodo <github@lazydodo.com>",
    ),
    "Red Mser <RedMser>": (
        "RedMser <RedMser>",
    ),
    "Richard Antalik <richardantalik@gmail.com>": (
        "Richard Antalik <ISS>",
        "Richard Antalik <iss@noreply.localhost>",
    ),
    "Robert Guetzkow <gitcommit@outlook.de>": (
        "Robert Guetzkow <rjg>",
    ),
    "Robin Hohnsbeen <robin@hohnsbeen.de>": (
        "Robin Hohnsbeen <robin-4@noreply.localhost>",
    ),
    "Sahar A. Kashi <sahar.alipourkashi@amd.com>": (
        "Sahar A. Kashi <salipour@noreply.localhost>",
    ),
    "Sebastian Herholz <sebastian.herholz@intel.com>": (
        "Sebastian Herholz <Sebastian.Herholz@gmail.com>",
        "Sebastian Herholz <sherholz>",
        "Sebastian Herholz <sherholz@noreply.localhost>",
        "Sebastian Herhoz <sebastian.herholz@intel.com>",
    ),
    "Sebastian Koenig <sebastiankoenig@posteo.de>": (
        "Sebastian Koenig <sebastian_k>",
        "Sebastian Koenig <sebastian_k@gmail.com>",
    ),
    "Sebastian Parborg <darkdefende@gmail.com>": (
        "Sebastian Parborg <zeddb>",
        "Sebastian Parborg <zeddb@noreply.localhost>",
        "Sebastian Ramacher <sramacher@debian.org>",
    ),
    "Sergey Sharybin <sergey@blender.org>": (
        "Sergey Sharybin <sergey.vfx@gmail.com>",
        "Sergey Sharybin <sergey>",
        "Sergey Sharybin <sergey@noreply.localhost>",
        "blender <sergey.vfx@gmail.com>",
    ),
    "Shane Ambler <Shane@ShaneWare.Biz>": (
        "Shane Ambler <shaneambler@noreply.localhost>",
    ),
    "Shashank Shekhar <secondary.cmdr2@gmail.com>": (
        "Shashank Shekhar <cmdr2>",
    ),
    "Siddhartha Jejurkar <f20180617@goa.bits-pilani.ac.in>": (
        "Siddhartha Jejurkar <sidd017>",
    ),
    "Sietse Brouwer <sietse@hetvrijeoog.nl>": (
        "Sietse Brouwer <SietseB>",
    ),
    "Simon G <intrigus>": (
        "Simon <intrigus>",
    ),
    "Sonny Campbell <sonny.campbell@unity3d.com>": (
        "DESKTOP-ON14TH5\\Sonny Campbell <sonny.campbell@unity3d.com>",
        "Sonny Campbell (@SonnyCampbell_Unity) <>",
        "Sonny Campbell <SonnyCampbell_Unity>",
        "Sonny Campbell <sonnycampbell_unity@noreply.localhost>",
    ),
    "Stefan Werner <stefan.werner@intel.com>": (
        "Stefan Werner <stefan.werner@tangent-animation.com>",
        "Stefan Werner <stefan@keindesign.de>",
        "Stefan Werner <stefan_werner@noreply.localhost>",
        "Stefan Werner <stewreo@gmail.com>",
        "Stefan Werner <swerner@smithmicro.com>",
        "Werner, Stefan <stefan.werner@intel.com>",
    ),
    "Stephan Seitz <theHamsta>": (
        "Stephan <theHamsta>",
    ),
    "Sybren A. Stüvel <sybren@blender.org>": (
        "Sybren A. StÃÂ¼vel <sybren@stuvel.eu>",
        "Sybren A. Stüvel <sybren>",
        "Sybren A. Stüvel <sybren@stuvel.eu>",
    ),
    "Thomas Dinges <thomas@blender.org>": (
        "Thomas Dinges <blender@dingto.org>",
        "Thomas Dinges <dingto>",
    ),
    "Thomas Szepe <HG1_public@gmx.net>": (
        "HG1 <HG1_public@gmx.net>",
    ),
    "Tom Edwards <contact@steamreview.org>": (
        "Tom Edwards <artfunkel>",
    ),
    "Tristan Porteries <republicthunderbolt9@gmail.com>": (
        "Porteries Tristan <republicthunderbolt9@gmail.com>",
    ),
    "Troy Sobotka <troy.sobotka@gmail.com>": (
        "Troy Sobotka <sobotka>",
    ),
    "Tuomo Keskitalo <tuomo.keskitalo@iki.fi>": (
        "Tuomo Keskitalo <tkeskita>",
    ),
    "Ulysse Martin <you.le@live.fr>": (
        "Ulysse Martin <youle>",
    ),
    "Vitor Boschi da Silva <vitorboschi>": (
        "Vitor Boschi <vitorboschi>",
    ),
    "Vuk Gardašević <lijenstina>": (
        "Vuk GardaÅ¡eviÄ <lijenstina>",
    ),
    "Wayde Moss <wbmoss_dev@yahoo.com>": (
        "Wayde Moss <GuiltyGhost>",
    ),
    "Weizhen Huang <weizhen@blender.org>": (
        "RiverIntheSky <itsnotrj@hotmail.com>",
        "Weizhen Huang <itsnotrj@gmail.com>",
        "Weizhen Huang <weizhen@noreply.localhost>",
    ),
    "William Leeson <william@blender.org>": (
        "William Leeson <leesonw>",
        "William Leeson <william.leeson@gmail.com>",
    ),
    "William Reynish <william@reynish.com>": (
        "William Reynish <billrey>",
        "William Reynish <billrey@me.com>",
        "William Reynish <billreynish>",
    ),
    "Willian Padovani Germano <wpgermano@gmail.com>": (
        "ianwill <wpgermano@gmail.com>",
    ),
    "Xavier Hallade <xavier.hallade@intel.com>": (
        "Xavier Hallade <xavierh@noreply.localhost>",
    ),
    "Yann Lanthony <yann-lty>": (
        "@yann-lty <>",
    ),
    "Yiming Wu <xp8110@outlook.com>": (
        "ChengduLittleA <xp8110@outlook.com>",
        "YimingWu <NicksBest>",
        "YimingWu <chengdulittlea@noreply.localhost>",
        "YimingWu <xp8110@outlook.com>",
        "YimingWu <xp8110t@outlook.com>",
    ),
    "jon denning <gfxcoder@gmail.com>": (
        "Jon Denning <gfxcoder>",
    ),
    "nBurn <nbwashburn@gmail.com>": (
        "nBurn <nBurn>",
    ),
    "nutti <nutti.metro@gmail.com>": (
        "nutti <Nutti>",
    ),
    "ok_what <ip1149a@gmail.com>": (
        "ok what <ok_what>",
    ),
}


# Some projects prefer not to have their developers listed.
author_table_exclude = {
    "Jason Fielder <jason-fielder@noreply.localhost>",
    "Matt McLin <mmclin@apple.com>",
    "Michael B Johnson <wave@noreply.localhost>",
    "Michael Jones <michael_p_jones@apple.com>",
    "Michael Parkin-White <mparkinwhite@apple.com>",
    "pwflocal <drwave@apple.com>",
}


def author_table_create() -> Dict[str, str]:
    import sys

    # Some validation:
    keys = list(author_table_source.keys())
    for i, (key, key_ordered) in enumerate(zip(keys, sorted(keys))):
        if key != key_ordered:
            print("Names unordered:", i)
            print(" ", key, "  ~ (found)")
            print(" ", key_ordered, "  ~ (expected)")
            sys.exit(1)

    for i, (key, value) in enumerate(author_table_source.items()):
        if value != tuple(sorted(value)):
            print("Name values:", key, "at index", i, "is not ordered")
            sys.exit(1)
        if len(set(value)) != len(value):
            print("Name values:", key, "at index", i, "contains duplicate values")
            sys.exit(1)
        if key in value:
            print("Name values:", key, "at index", i, "contains the key in the values body")
            sys.exit(1)

    table = {}
    for key, values in author_table_source.items():
        for value_old in values:
            table[value_old] = key
    return table


author_table = author_table_create()

# Mapping from a comit hash to additional authors.
# Fully overwrite authors gathered from git commit info.
# Intended usage: Correction of info stored in git commit itself.
# Note that the names of the authors here are assumed fully valid and usable as-is.
commit_authors_overwrite: Dict[bytes, Tuple[str, ...]] = {
    # Format: {full_git_hash: (tuple, of, authors),}.
    # Author was: `blender <blender@localhost.localdomain>`.
    b"ba3d49225c9ff3514fb87ae5d692baefe5edec30": ("Sergey Sharybin <sergey@blender.org>", ),
    # Author was: `Author Name <email@address.com>`.
    b"4b6a4b5bc25bce10367dffadf7718e373f81f299": ("Antonio Vazquez <blendergit@gmail.com>", ),
}


# -----------------------------------------------------------------------------
# Multi-Processing

def process_commits_for_map(commits: Iterable[GitCommit]) -> "Credits":
    result = Credits()
    for c in commits:
        result.process_commit(c)
    return result


# -----------------------------------------------------------------------------
# Class for generating authors


class CreditUser:
    __slots__ = (
        "commit_total",
        "lines_change",
    )

    def __init__(self) -> None:
        self.commit_total = 0
        self.lines_change = -1


class Credits:
    __slots__ = (
        "users",
    )

    # Expected to cover the following formats (the e-mail address is not captured if present):
    #    `Co-authored-by: Blender Foundation`
    #    `Co-authored-by: Blender Foundation <foundation@blender.org>`
    #    `Co-authored-by: Blender Foundation <Suzanne>`
    GIT_COMMIT_COAUTHORS_RE = re.compile(r"^Co-authored-by:[ \t]*([^\n]+)$", re.MULTILINE)

    def __init__(self) -> None:
        self.users: Dict[str, CreditUser] = {}

    @classmethod
    def commit_authors_get(cls, c: GitCommit) -> List[str]:
        if (authors_overwrite := commit_authors_overwrite.get(c.sha1, None)) is not None:
            # Ignore git commit info for these having an entry in commit_authors_overwrite.
            return [author_table.get(author, author) for author in authors_overwrite]

        # Normalize author string into canonical form, prevents duplicate credit users
        authors = [unicodedata.normalize('NFC', "{:s} <{:s}>".format(c.author, c.email))]
        for author in cls.GIT_COMMIT_COAUTHORS_RE.findall(c.body):
            author = unicodedata.normalize('NFC', author)
            if not ("<" in author and ">" in author):
                author = author + " <>"
            authors.append(author)

        return [author_table.get(author, author) for author in authors]

    @classmethod
    def is_credit_commit_valid(cls, c: GitCommit) -> bool:
        ignore_dir = (
            b"blender/extern/",
            b"blender/intern/opennl/",
            # Will have own authors file.
            b"intern/cycles/",
            # Will have own authors file.
            b"intern/libmv/libmv/"
        )

        if not any(f for f in c.files if not f.startswith(ignore_dir)):
            return False

        return True

    def merge(self, other: "Credits") -> None:
        """
        Merge other Credits into this, clearing the other.
        """
        for user_key, user_other in other.users.items():
            user = self.users.get(user_key)
            if user is None:
                # Consume the user.
                self.users[user_key] = user_other
            else:
                user.commit_total += user_other.commit_total
                user.lines_change += user_other.lines_change
        other.users.clear()

    def process_commit(self, c: GitCommit) -> None:
        if not self.is_credit_commit_valid(c):
            return

        lines_change = -1

        authors = self.commit_authors_get(c)
        year = c.date.year
        for author in authors:
            cu = self.users.get(author)
            if cu is None:
                cu = self.users[author] = CreditUser()

            cu.commit_total += 1

            lines_change_for_author = cu.lines_change
            if lines_change_for_author < AUTHOR_LINES_LIMIT:
                if lines_change_for_author == -1:
                    cu.lines_change = lines_change_for_author = 0

                if lines_change == -1:
                    diff = c.diff
                    lines_change = diff.count("\n-") + diff.count("\n+")

                cu.lines_change += lines_change

    def _process_multiprocessing(self, commit_iter: Iterable[GitCommit], *, jobs: int) -> None:
        print("Collecting commits...")
        # NOTE(@ideasman42): that the chunk size doesn't have as much impact on
        # performance as you might expect, values between 16 and 1024 seem reasonable.
        # Although higher values tend to bottleneck as the process finishes.
        chunk_size = 256
        chunk_list = []
        chunk = []
        for i, c in enumerate(commit_iter):
            chunk.append(c)
            if len(chunk) >= chunk_size:
                chunk_list.append(chunk)
                chunk = []
        if chunk:
            chunk_list.append(chunk)

        total_commits = (max(len(chunk_list) - 1, 0) * chunk_size) + len(chunk)

        print("Found {:d} commits, processing...".format(total_commits))
        with multiprocessing.Pool(processes=jobs) as pool:
            for i, result in enumerate(pool.imap_unordered(process_commits_for_map, chunk_list)):
                print("{:d} of {:d}".format(i, len(chunk_list)))
                self.merge(result)

    def process(self, commit_iter: Iterable[GitCommit], *, jobs: int) -> None:
        if jobs > 1:
            self._process_multiprocessing(commit_iter, jobs=jobs)
            return

        # Simple single process operation.
        for i, c in enumerate(commit_iter):
            self.process_commit(c)
            if not (i % 100):
                print(i)

    def write(
            self,
            filepath: str,
            use_metadata: bool = False,
    ) -> None:

        # patch_word = "patch", "patches"
        commit_word = "commit", "commits"

        sorted_authors = dict(sorted(self.users.items()))

        right_margin = 79
        with open(filepath, 'w', encoding="utf8", errors='xmlcharrefreplace') as file:
            for author_with_email, cu in sorted_authors.items():
                if author_with_email.endswith(" <>"):
                    print("Skipping:", author_with_email, "(no email)")
                    continue
                if cu.lines_change <= AUTHOR_LINES_SKIP:
                    print("Skipping:", author_with_email, cu.lines_change, "line(s) changed.")
                    continue
                if author_with_email in author_table_exclude:
                    print("Skipping:", author_with_email, "explicit exclusion requested.")
                    continue

                if use_metadata:
                    file.write("{:s} {:s}# lines={:,d} ({:s}), {:,d} {:s}\n".format(
                        author_with_email,
                        (" " * max(1, right_margin - len(author_with_email))),
                        min(cu.lines_change, AUTHOR_LINES_LIMIT),
                        "" if cu.lines_change >= AUTHOR_LINES_LIMIT else "<SKIP?>",
                        cu.commit_total,
                        commit_word[cu.commit_total > 1],
                    ))
                else:
                    file.write("{:s}\n".format(author_with_email))


def argparse_create() -> argparse.ArgumentParser:

    # When --help or no args are given, print this help
    usage_text = "List authors."

    epilog = "This script is used to generate an AUTHORS file"

    parser = argparse.ArgumentParser(description=usage_text, epilog=epilog)

    parser.add_argument(
        "--source", dest="source_dir",
        metavar='PATH',
        required=True,
        help="Path to git repository",
    )
    parser.add_argument(
        "--range",
        dest="range_sha1",
        metavar='SHA1_RANGE',
        required=True,
        help="Range to use, eg: 169c95b8..HEAD",
    )

    parser.add_argument(
        "--jobs",
        dest="jobs",
        type=int,
        default=0,
        help=(
            "The number of processes to use. "
            "Defaults to zero which detects the available cores, 1 is single threaded (useful for debugging)."
        ),
        required=False,
    )

    return parser


def main() -> None:

    # ----------
    # Parse Args

    args = argparse_create().parse_args()

    credits = Credits()
    # commit_range = "HEAD~10..HEAD"
    # commit_range = "blender-v2.81-release..blender-v2.82-release"
    # commit_range = "blender-v2.82-release"
    commit_range = args.range_sha1
    jobs = args.jobs
    if jobs <= 0:
        # Clamp the value, higher values give errors with too many open files.
        # Allow users to manually pass very high values in as they might want to tweak system limits themselves.
        jobs = min(multiprocessing.cpu_count() * 2, 400)

    credits.process(GitCommitIter(args.source_dir, commit_range), jobs=jobs)

    credits.write("AUTHORS_UPDATE")
    print("Written: AUTHORS_UPDATE")


if __name__ == "__main__":
    main()
