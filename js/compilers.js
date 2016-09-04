(function() {
  this.COMPILERS = [
    {
      name: 'Haxe ActionScript Compiler',
      source: 'Haxe',
      target: 'ActionScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Haxe C# Compiler',
      source: 'Haxe',
      target: 'C#',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Haxe C++ Compiler',
      source: 'Haxe',
      target: 'C++',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Haxe Java Compiler',
      source: 'Haxe',
      target: 'Java',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Haxe Lua Compiler',
      source: 'Haxe',
      target: 'Lua',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Haxe PHP Compiler',
      source: 'Haxe',
      target: 'PHP',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Haxe Python Compiler',
      source: 'Haxe',
      target: 'Python',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Js2Py',
      source: 'JavaScript',
      target: 'Python',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Bridge.NET',
      source: 'C#',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Brython',
      source: 'Python',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'BuckleScript',
      source: 'OCaml',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Ceylon JavaScript Compiler',
      source: 'Ceylon',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'ClojureScript',
      source: 'Clojure',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'CoffeeScript Compiler',
      source: 'CoffeeScript',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Dart-to-JavaScript Compiler',
      source: 'Dart',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Elm Compiler',
      source: 'Elm',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Emscripten',
      source: 'LLVM IR',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Fable',
      source: 'F#',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'GHCJS',
      source: 'Haskell',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'GopherJS',
      source: 'Go',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Haxe JavaScript Compiler',
      source: 'Haxe',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'JSIL',
      source: 'CIL',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'JSweet',
      source: 'Java',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Kotlin JavaScript Compiler',
      source: 'Kotlin',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Scala.js',
      source: 'Scala',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'TeaVM',
      source: 'Java Bytecode',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Transcrypt',
      source: 'Python',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'TypeScript Compiler',
      source: 'TypeScript',
      target: 'JavaScript',
      type: 'Transpiler',
      url: 'none'
    }, {
      name: 'Clang C Compiler',
      source: 'C',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Clang C++ Compiler',
      source: 'C++',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Clang Objective-C Compiler',
      source: 'Objective-C',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'LDC',
      source: 'D',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Nim Compiler',
      source: 'Nim',
      target: 'C',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Rust Compiler',
      source: 'Rust',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Swift Compiler',
      source: 'Swift',
      target: 'LLVM IR',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'IronPython',
      source: 'Python',
      target: 'CIL',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Roslyn',
      source: 'C#',
      target: 'CIL',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Visual F#',
      source: 'F#',
      target: 'CIL',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Ceylon Compiler',
      source: 'Ceylon',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'none'
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
      url: 'none'
    }, {
      name: 'Jython',
      source: 'Python',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Kotlin Compiler',
      source: 'Kotlin',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'Scala Compiler',
      source: 'Scala',
      target: 'Java Bytecode',
      type: 'Intermediate',
      url: 'none'
    }, {
      name: 'DMD',
      source: 'D',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'FreeBASIC',
      source: 'BASIC',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'Free Pascal',
      source: 'Pascal',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'GCC',
      source: 'C',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'GFortran',
      source: 'Fortran',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'GHC',
      source: 'Haskell',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'GNAT',
      source: 'Ada',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'Go Compiler',
      source: 'Go',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'G++',
      source: 'C++',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'Intel C',
      source: 'C',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'Intel C++',
      source: 'C++',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'LLVM',
      source: 'LLVM IR',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'OCaml Compiler',
      source: 'OCaml',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'Visual C',
      source: 'C',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'Visual C++',
      source: 'C++',
      target: 'Machine Code',
      type: 'Native',
      url: 'none'
    }, {
      name: 'CLR',
      source: 'CIL',
      target: 'Machine Code',
      type: 'JIT',
      url: 'none'
    }, {
      name: 'HHVM Hack',
      source: 'Hack',
      target: 'Machine Code',
      type: 'JIT',
      url: 'none'
    }, {
      name: 'HHVM PHP',
      source: 'PHP',
      target: 'Machine Code',
      type: 'JIT',
      url: 'none'
    }, {
      name: 'JVM',
      source: 'Java Bytecode',
      target: 'Machine Code',
      type: 'JIT',
      url: 'none'
    }, {
      name: 'LuaJIT',
      source: 'Lua',
      target: 'Machine Code',
      type: 'JIT',
      url: 'none'
    }, {
      name: 'PyPy',
      source: 'Python',
      target: 'Machine Code',
      type: 'JIT',
      url: 'none'
    }, {
      name: 'V8',
      source: 'JavaScript',
      target: 'Machine Code',
      type: 'JIT',
      url: 'none'
    }, {
      name: 'MASM',
      source: 'Assembly',
      target: 'Machine Code',
      type: 'Assembler',
      url: 'none'
    }, {
      name: 'NASM',
      source: 'Assembly',
      target: 'Machine Code',
      type: 'Assembler',
      url: 'none'
    }
  ];

}).call(this);
