use proc_macro::TokenStream;
use quote::quote;
use std::sync::atomic::AtomicU64;
use std::sync::Mutex;
use syn::ItemStruct;

static UNIQUE_COUNTER: Mutex<AtomicU64> = Mutex::new(AtomicU64::new(0));

#[proc_macro_attribute]
pub fn manifold_warp(_attr: TokenStream, input: TokenStream) -> TokenStream {
    let structt = syn::parse_macro_input!(input as ItemStruct);
    let struct_ident = structt.ident.clone();
    let struct_name = struct_ident.to_string();

    let unique_id = match UNIQUE_COUNTER.lock() {
        Ok(guard) => guard.fetch_add(1, std::sync::atomic::Ordering::SeqCst),
        Err(e) => panic!("Could not lock unique counter: {}", e),
    };

    let extern_c_fn_ident = proc_macro2::Ident::new(
        format!(
            "manifold3d_manifold_warp_fn_{}_{}",
            struct_name.to_ascii_lowercase(),
            unique_id
        )
            .as_str(),
        proc_macro2::Span::call_site(),
    );

    let output = quote!(
        #structt

        const _: () = {
            use manifold3d::manifold::WarpVertex;

            #[no_mangle]
            #[doc(hidden)]
            pub unsafe extern "C" fn #extern_c_fn_ident(
                x: f64,
                y: f64,
                z: f64,
                ctx: *mut ::std::os::raw::c_void
            ) -> manifold3d::sys::ManifoldVec3 {
                let warp = &*(ctx as *mut #struct_ident);
                let result = warp.warp_vertex(manifold3d::types::Point3::new(x, y, z));
                result.into()
            }

            #[automatically_derived]
            impl manifold3d::manifold::ExternCWarpFn for #struct_ident {
                fn extern_c_warp_fn(&self) -> unsafe extern "C" fn(
                    f64,
                    f64,
                    f64,
                    *mut std::os::raw::c_void
                ) -> manifold3d::sys::ManifoldVec3 {
                    #extern_c_fn_ident
                }
            }
        };

        #[automatically_derived]
        impl manifold3d::manifold::Warp for #struct_ident {}
    );
    output.into()
}

#[proc_macro_attribute]
pub fn manifold_manage_vertex_properties(_attr: TokenStream, input: TokenStream) -> TokenStream {
    let structt = syn::parse_macro_input!(input as ItemStruct);
    let struct_ident = structt.ident.clone();
    let struct_name = struct_ident.to_string();

    let unique_id = match UNIQUE_COUNTER.lock() {
        Ok(guard) => guard.fetch_add(1, std::sync::atomic::Ordering::SeqCst),
        Err(e) => panic!("Could not lock unique counter: {}", e),
    };

    let extern_c_fn_ident = proc_macro2::Ident::new(
        format!(
            "manifold3d_manifold_replace_vertex_properties_fn_{}_{}",
            struct_name.to_ascii_lowercase(),
            unique_id
        )
            .as_str(),
        proc_macro2::Span::call_site(),
    );

    let output = quote!(
        #structt

        const _: () = {
            use manifold3d::manifold::ReplaceVertexProperties;
            #[no_mangle]
            #[doc(hidden)]
            pub unsafe extern "C" fn #extern_c_fn_ident(
                new_prop: *mut f64,
                position: manifold3d_sys::ManifoldVec3,
                old_prop: *const f64,
                ctx: *mut ::std::os::raw::c_void,
            ) {
                let c_ctx = &*(ctx as *const manifold3d::manifold::ReplaceVertexPropertiesCCtx);
                let manage_vertex_properties =
                    &*(c_ctx.manage_vertex_properties_ptr as *const #struct_ident);

                // We cannot access the concrete type of CTX here, so we let the compiler infer the correct type
                unsafe fn convert_ctx<'a, T>(ctx: *mut ::std::os::raw::c_void) -> &'a mut T {
                    &mut *(ctx as *mut T)
                }
                let manage_vertex_properties_ctx = convert_ctx(c_ctx.ctx_ptr);

                let old_properties = if old_prop.is_null() {
                    &[0.0; 0]
                } else {
                    std::slice::from_raw_parts(old_prop, c_ctx.old_properties_per_vertex_count)
                };

                let mut new_properties = Vec::from_raw_parts(
                    new_prop,
                    c_ctx.new_properties_per_vertex_count,
                    c_ctx.new_properties_per_vertex_count,
                );

                manage_vertex_properties.replace_vertex_properties(
                    manage_vertex_properties_ctx,
                    position.into(),
                    old_properties,
                    new_properties.as_mut_slice(),
                );

                // Managed by manifold
                ::std::mem::forget(new_properties);
            }

            #[automatically_derived]
            impl manifold3d::manifold::ExternCReplaceVertexPropertiesFn for MyPropertyReplacer {
                fn extern_c_replace_vertex_properties_fn(
                    &self,
                ) -> unsafe extern "C" fn(
                    *mut f64,
                    manifold3d_sys::ManifoldVec3,
                    *const f64,
                    *mut ::std::os::raw::c_void,
                ) -> () {
                    #extern_c_fn_ident
                }
            }
        };

        #[automatically_derived]
        impl manifold3d::manifold::ManageVertexProperties for MyPropertyReplacer {}
    );
    output.into()
}