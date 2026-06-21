//! Derive macro for `nfe_core::params::Tunable`.
//!
//! Reflects a parameter struct into a flat, namespaced, bounded search space so
//! that both config loading and black-box optimizers (CMA-ES / Optuna / SMAC)
//! consume one source of truth. Continuous and integer specs are distinguished
//! so mixed optimizers can treat window sizes / counts as discrete.
//!
//! Field attributes
//! ────────────────
//!   #[param(0.0..4.0, default = 0.80)]        continuous, linear scale
//!   #[param(1e-3..1e1, default = 0.05, log)]  continuous, log scale
//!   #[param(int, 1..21, default = 5)]         integer (inclusive-exclusive hi)
//!   #[tunable(nested)]                        recurse into the field's Tunable
//!   #[tunable(skip)]                          excluded; rebuilt via Default
//!
//! The generated impl namespaces every descriptor as "<prefix>.<field>" so a
//! top-level `Config` can flatten the whole tree with stable, dotted keys.

use proc_macro::TokenStream;
use proc_macro2::TokenStream as TokenStream2;
use quote::quote;
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Data, DeriveInput, Expr, Fields, Ident, Token, Type};

/// One field's parsed `#[param(...)]` body.
struct ParamAttr {
    integer: bool,
    log: bool,
    lo: Expr,
    hi: Expr,
    default: Expr,
}

impl Parse for ParamAttr {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut integer = false;
        let mut log = false;
        let mut lo: Option<Expr> = None;
        let mut hi: Option<Expr> = None;
        let mut default: Option<Expr> = None;

        while !input.is_empty() {
            // `default = <expr>`
            if input.peek(Ident) && input.peek2(Token![=]) {
                let key: Ident = input.parse()?;
                let _eq: Token![=] = input.parse()?;
                let val: Expr = input.parse()?;
                if key == "default" {
                    default = Some(val);
                } else {
                    return Err(syn::Error::new_spanned(key, "unknown param key"));
                }
            } else if input.peek(Ident) {
                // bare flag: `int` or `log`
                let flag: Ident = input.parse()?;
                if flag == "int" {
                    integer = true;
                } else if flag == "log" {
                    log = true;
                } else {
                    return Err(syn::Error::new_spanned(flag, "expected `int` or `log`"));
                }
            } else {
                // range: syn parses `lo .. hi` as a single Expr::Range.
                let expr: Expr = input.parse()?;
                match expr {
                    Expr::Range(r) => {
                        let l = *r.start.ok_or_else(|| {
                            syn::Error::new_spanned(r.limits, "missing range lower bound")
                        })?;
                        let h = *r.end.ok_or_else(|| {
                            syn::Error::new_spanned(r.limits, "missing range upper bound")
                        })?;
                        lo = Some(l);
                        hi = Some(h);
                    }
                    other => {
                        return Err(syn::Error::new_spanned(other, "expected a `lo..hi` range"))
                    }
                }
            }

            if input.peek(Token![,]) {
                let _comma: Token![,] = input.parse()?;
            }
        }

        let lo = lo.ok_or_else(|| input.error("missing range lower bound"))?;
        let hi = hi.ok_or_else(|| input.error("missing range upper bound"))?;
        let default = default.ok_or_else(|| input.error("missing `default = ...`"))?;
        Ok(ParamAttr {
            integer,
            log,
            lo,
            hi,
            default,
        })
    }
}

enum FieldKind {
    Param(Box<ParamAttr>),
    Nested,
    Skip,
}

fn classify(field: &syn::Field) -> syn::Result<FieldKind> {
    for attr in &field.attrs {
        if attr.path().is_ident("param") {
            let parsed: ParamAttr = attr.parse_args()?;
            return Ok(FieldKind::Param(Box::new(parsed)));
        }
        if attr.path().is_ident("tunable") {
            // #[tunable(nested)] | #[tunable(skip)]
            let ident: Ident = attr.parse_args()?;
            if ident == "nested" {
                return Ok(FieldKind::Nested);
            } else if ident == "skip" {
                return Ok(FieldKind::Skip);
            }
            return Err(syn::Error::new_spanned(
                ident,
                "expected `nested` or `skip`",
            ));
        }
    }
    // Default: a field with no annotation is excluded from the search space.
    Ok(FieldKind::Skip)
}

#[proc_macro_derive(Tunable, attributes(param, tunable))]
pub fn derive_tunable(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    let fields = match &input.data {
        Data::Struct(s) => match &s.fields {
            Fields::Named(named) => &named.named,
            _ => {
                return syn::Error::new_spanned(name, "Tunable requires named fields")
                    .to_compile_error()
                    .into()
            }
        },
        _ => {
            return syn::Error::new_spanned(name, "Tunable can only derive on structs")
                .to_compile_error()
                .into()
        }
    };

    let mut descriptor_stmts: Vec<TokenStream2> = Vec::new();
    let mut to_flat_stmts: Vec<TokenStream2> = Vec::new();
    let mut from_flat_inits: Vec<TokenStream2> = Vec::new();

    for field in fields {
        let ident = field.ident.as_ref().unwrap();
        let ty: &Type = &field.ty;
        let key = ident.to_string();

        match classify(field) {
            Ok(FieldKind::Param(p)) => {
                let ParamAttr {
                    integer,
                    log,
                    lo,
                    hi,
                    default,
                } = *p;

                if integer {
                    descriptor_stmts.push(quote! {
                        let __k = if prefix.is_empty() {
                            #key.to_string()
                        } else {
                            format!("{}.{}", prefix, #key)
                        };
                        out.push((
                            __k,
                            ::nfe_core::params::ParamSpec::Integer {
                                lo: (#lo) as i64,
                                hi: (#hi) as i64,
                                default: (#default) as i64,
                            },
                        ));
                    });
                    // Missing keys fall back to the declared default so partial
                    // search spaces (tuning a subset) still reconstruct fully.
                    from_flat_inits.push(quote! {
                        #ident: {
                            let __k = if prefix.is_empty() {
                                #key.to_string()
                            } else {
                                format!("{}.{}", prefix, #key)
                            };
                            let __v = values.get(&__k).copied().unwrap_or((#default) as f64);
                            __v.round() as #ty
                        },
                    });
                } else {
                    descriptor_stmts.push(quote! {
                        let __k = if prefix.is_empty() {
                            #key.to_string()
                        } else {
                            format!("{}.{}", prefix, #key)
                        };
                        out.push((
                            __k,
                            ::nfe_core::params::ParamSpec::Continuous {
                                lo: (#lo) as f64,
                                hi: (#hi) as f64,
                                default: (#default) as f64,
                                log: #log,
                            },
                        ));
                    });
                    from_flat_inits.push(quote! {
                        #ident: {
                            let __k = if prefix.is_empty() {
                                #key.to_string()
                            } else {
                                format!("{}.{}", prefix, #key)
                            };
                            let __v = values.get(&__k).copied().unwrap_or((#default) as f64);
                            __v as #ty
                        },
                    });
                }

                to_flat_stmts.push(quote! {
                    let __k = if prefix.is_empty() {
                        #key.to_string()
                    } else {
                        format!("{}.{}", prefix, #key)
                    };
                    out.push((__k, self.#ident as f64));
                });
            }
            Ok(FieldKind::Nested) => {
                descriptor_stmts.push(quote! {
                    let __prefix = if prefix.is_empty() {
                        #key.to_string()
                    } else {
                        format!("{}.{}", prefix, #key)
                    };
                    <#ty as ::nfe_core::params::Tunable>::descriptors(&__prefix, out);
                });
                to_flat_stmts.push(quote! {
                    let __prefix = if prefix.is_empty() {
                        #key.to_string()
                    } else {
                        format!("{}.{}", prefix, #key)
                    };
                    self.#ident.to_flat(&__prefix, out);
                });
                from_flat_inits.push(quote! {
                    #ident: {
                        let __prefix = if prefix.is_empty() {
                            #key.to_string()
                        } else {
                            format!("{}.{}", prefix, #key)
                        };
                        <#ty as ::nfe_core::params::Tunable>::from_flat(&__prefix, values)
                    },
                });
            }
            Ok(FieldKind::Skip) => {
                from_flat_inits.push(quote! {
                    #ident: ::core::default::Default::default(),
                });
            }
            Err(e) => return e.to_compile_error().into(),
        }
    }

    let expanded = quote! {
        impl ::nfe_core::params::Tunable for #name {
            fn descriptors(prefix: &str, out: &mut ::std::vec::Vec<(::std::string::String, ::nfe_core::params::ParamSpec)>) {
                #(#descriptor_stmts)*
            }

            fn to_flat(&self, prefix: &str, out: &mut ::std::vec::Vec<(::std::string::String, f64)>) {
                #(#to_flat_stmts)*
            }

            fn from_flat(prefix: &str, values: &::std::collections::HashMap<::std::string::String, f64>) -> Self {
                Self {
                    #(#from_flat_inits)*
                }
            }
        }
    };

    expanded.into()
}
