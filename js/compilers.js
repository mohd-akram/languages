(function() {
  this.COMPILERS = [
    {
      name: 'Haxe ActionScript Compiler',
      source: 'Haxe',
      target: 'ActionScript',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Haxe C# Compiler',
      source: 'Haxe',
      target: 'C#',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Haxe C++ Compiler',
      source: 'Haxe',
      target: 'C++',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Haxe Java Compiler',
      source: 'Haxe',
      target: 'Java',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Haxe JavaScript Compiler',
      source: 'Haxe',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Haxe Lua Compiler',
      source: 'Haxe',
      target: 'Lua',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Haxe PHP Compiler',
      source: 'Haxe',
      target: 'PHP',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Haxe Python Compiler',
      source: 'Haxe',
      target: 'Python',
      type: 'Transpiler',
      url: 'http://haxe.org/'
    }, {
      name: 'Js2Py',
      source: 'JavaScript',
      target: 'Python',
      type: 'Transpiler',
      url: 'https://piotr-dabkowski.appspot.com/projects'
    }, {
      name: 'Bridge.NET',
      source: 'C#',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://bridge.net/'
    }, {
      name: 'Brython',
      source: 'Python',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'https://www.brython.info/'
    }, {
      name: 'BuckleScript',
      source: 'OCaml',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://bloomberg.github.io/bucklescript/'
    }, {
      name: 'Ceylon JavaScript Compiler',
      source: 'Ceylon',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://ceylon-lang.org/'
    }, {
      name: 'ClojureScript',
      source: 'Clojure',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://clojurescript.org/'
    }, {
      name: 'CoffeeScript Compiler',
      source: 'CoffeeScript',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://coffeescript.org/'
    }, {
      name: 'Dart-to-JavaScript Compiler',
      source: 'Dart',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'https://www.dartlang.org/'
    }, {
      name: 'Elm Compiler',
      source: 'Elm',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://elm-lang.org/'
    }, {
      name: 'Emscripten',
      source: 'LLVM IR',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://emscripten.org'
    }, {
      name: 'Fable',
      source: 'F#',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'https://fable-compiler.github.io/'
    }, {
      name: 'GHCJS',
      source: 'Haskell',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'https://github.com/ghcjs'
    }, {
      name: 'GopherJS',
      source: 'Go',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://www.gopherjs.org/'
    }, {
      name: 'JSIL',
      source: 'CIL',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://jsil.org/'
    }, {
      name: 'JSweet',
      source: 'Java',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://www.jsweet.org/'
    }, {
      name: 'Kotlin JavaScript Compiler',
      source: 'Kotlin',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'https://kotlinlang.org/'
    }, {
      name: 'Scala.js',
      source: 'Scala',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'https://www.scala-js.org/'
    }, {
      name: 'TeaVM',
      source: 'Java Bytecode',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://teavm.org/'
    }, {
      name: 'Transcrypt',
      source: 'Python',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'http://transcrypt.org/'
    }, {
      name: 'TypeScript Compiler',
      source: 'TypeScript',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'https://www.typescriptlang.org/'
    }, {
      name: 'Clang C Compiler',
      source: 'C',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'http://clang.llvm.org/'
    }, {
      name: 'Clang C++ Compiler',
      source: 'C++',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'http://clang.llvm.org/'
    }, {
      name: 'Clang Objective-C Compiler',
      source: 'Objective-C',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'http://clang.llvm.org/'
    }, {
      name: 'LDC',
      source: 'D',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'http://wiki.dlang.org/LDC'
    }, {
      name: 'Nim Compiler',
      source: 'Nim',
      target: 'C',
      type: 'Intermediate',
      url: 'http://nim-lang.org/'
    }, {
      name: 'Rust Compiler',
      source: 'Rust',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'https://www.rust-lang.org/'
    }, {
      name: 'Swift Compiler',
      source: 'Swift',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'https://swift.org/'
    }, {
      name: 'IronPython',
      source: 'Python',
      target: 'CIL',
      type: 'Intermediate',
      url: 'http://ironpython.net/'
    }, {
      name: 'Roslyn',
      source: 'C#',
      target: 'CIL',
      type: 'Intermediate',
      url: 'https://github.com/dotnet/roslyn'
    }, {
      name: 'Visual F#',
      source: 'F#',
      target: 'CIL',
      type: 'Intermediate',
      url: 'https://msdn.microsoft.com/en-us/visualfsharpdocs/conceptual/visual-fsharp'
    }, {
      name: 'Ceylon Compiler',
      source: 'Ceylon',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'http://ceylon-lang.org/'
    }, {
      name: 'Clojure Compiler',
      source: 'Clojure',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Groovy Compiler',
      source: 'Groovy',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'http://groovy-lang.org/groovyc.html'
    }, {
      name: 'Java Compiler',
      source: 'Java',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'https://en.wikipedia.org/wiki/Java_compiler'
    }, {
      name: 'Jython',
      source: 'Python',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'http://www.jython.org/'
    }, {
      name: 'Kotlin Compiler',
      source: 'Kotlin',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'https://kotlinlang.org/'
    }, {
      name: 'Scala Compiler',
      source: 'Scala',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'http://www.scala-lang.org/'
    }, {
      name: 'DMD',
      source: 'D',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://tour.dlang.org/'
    }, {
      name: 'FreeBASIC',
      source: 'BASIC',
      target: 'Machine Code',
      type: 'Native',
      url: 'http://www.freebasic.net/'
    }, {
      name: 'Free Pascal',
      source: 'Pascal',
      target: 'Machine Code',
      type: 'Native',
      url: 'http://www.freepascal.org/'
    }, {
      name: 'GCC',
      source: 'C',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://gcc.gnu.org/'
    }, {
      name: 'GFortran',
      source: 'Fortran',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://gcc.gnu.org/fortran/'
    }, {
      name: 'GHC',
      source: 'Haskell',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://www.haskell.org/'
    }, {
      name: 'GNAT',
      source: 'Ada',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://www.gnu.org/software/gnat/'
    }, {
      name: 'Go Compiler',
      source: 'Go',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://golang.org/'
    }, {
      name: 'G++',
      source: 'C++',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://gcc.gnu.org/'
    }, {
      name: 'Intel C',
      source: 'C',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://software.intel.com/en-us/c-compilers'
    }, {
      name: 'Intel C++',
      source: 'C++',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://software.intel.com/en-us/c-compilers'
    }, {
      name: 'LLVM',
      source: 'LLVM IR',
      target: 'Machine Code',
      type: 'Native',
      url: 'http://llvm.org/'
    }, {
      name: 'OCaml Compiler',
      source: 'OCaml',
      target: 'Machine Code',
      type: 'Native',
      url: 'https://ocaml.org/'
    }, {
      name: 'Visual C',
      source: 'C',
      target: 'Machine Code',
      type: 'Native',
      url: 'http://landinghub.visualstudio.com/visual-cpp-build-tools'
    }, {
      name: 'Visual C++',
      source: 'C++',
      target: 'Machine Code',
      type: 'Native',
      url: 'http://landinghub.visualstudio.com/visual-cpp-build-tools'
    }, {
      name: 'CLR',
      source: 'CIL',
      target: 'Machine Code',
      type: 'JIT',
      url: 'https://msdn.microsoft.com/en-us/library/k8d11d4s.aspx'
    }, {
      name: 'HHVM Hack',
      source: 'Hack',
      target: 'Machine Code',
      type: 'JIT',
      url: 'http://hhvm.com/'
    }, {
      name: 'HHVM PHP',
      source: 'PHP',
      target: 'Machine Code',
      type: 'JIT',
      url: 'http://hhvm.com/'
    }, {
      name: 'JVM',
      source: 'Java Bytecode',
      target: 'Machine Code',
      type: 'JIT',
      url: 'https://en.wikipedia.org/wiki/Java_virtual_machine'
    }, {
      name: 'LuaJIT',
      source: 'Lua',
      target: 'Machine Code',
      type: 'JIT',
      url: 'http://luajit.org/'
    }, {
      name: 'PyPy',
      source: 'Python',
      target: 'Machine Code',
      type: 'JIT',
      url: 'http://pypy.org/'
    }, {
      name: 'V8',
      source: 'JavaScript',
      target: 'Machine Code',
      type: 'JIT',
      url: 'https://developers.google.com/v8/'
    }, {
      name: 'MASM',
      source: 'Assembly',
      target: 'Machine Code',
      type: 'Assembler',
      url: 'http://www.masm32.com/'
    }, {
      name: 'NASM',
      source: 'Assembly',
      target: 'Machine Code',
      type: 'Assembler',
      url: 'http://www.nasm.us/'
    }, {
      name: 'YASM',
      source: 'Assembly',
      target: 'Machine Code',
      type: 'Assembler',
      url: 'http://yasm.tortall.net/'
    }
  ];

}).call(this);
