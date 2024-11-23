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
            "manifold3d_manifold_extern_c_warp_fn_{}_{}",
            struct_name.to_ascii_lowercase(),
            unique_id
        )
        .as_str(),
        proc_macro2::Span::call_site(),
    );

    let output = quote!(
        #structt

        const _: () = {
            use manifold3d::types::manifold::vertex::WarpImpl;

            #[no_mangle]
            #[doc(hidden)]
            pub unsafe extern "C" fn #extern_c_fn_ident(
                x: f64,
                y: f64,
                z: f64,
                ctx: *mut ::std::os::raw::c_void
            ) -> manifold3d::sys::ManifoldVec3 {
                let warp = &*(ctx as *mut #struct_ident);
                let result = warp.warp_vertex(manifold3d::types::math::Point3::new(x, y, z));
                result.into()
            }

            #[automatically_derived]
            impl manifold3d::types::manifold::vertex::ExternCWarpFn for #struct_ident {
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
        impl manifold3d::types::manifold::vertex::Warp for #struct_ident {}
    );
    output.into()
}
